#!/usr/bin/env python3

# Inclusão de bibliotecas
import time 
import copy 
import csv
import os
from datetime import datetime

import rclpy 
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

# Importação dos seus módulos locais
from RoboticSorter import RoboticSorter 
from Controller_CR3 import Controller_CR3, execute_move_sequence

# Importa as constantes
from constants import (
    ROBOT_POINTS_DATABASE, 
    POINT_MEASURE, 
    POINT_MEASURE_CARTESIAN, 
    POINT_TAKE_BOX, 
    SLOT_POINTS, 
    POINT_CHANGE_SLOT 
)

# --- CONFIGURAÇÃO DE ESCALA E CALIBRAÇÃO ---
Z_OFFSET_SAFETY = 65.0 # Margem de segurança em mm para não esmagar a caixa

# --- CALIBRAÇÃO DE ALTURA ---
# Leitura do LiDAR (em metros) para a caixa padrão de 10cm
LIDAR_Z_REFERENCE_10CM = 0.26738 


def apply_z_offset(db_original, offset_z_mm):
    """
    Cria uma cópia do banco de dados de pontos e aplica o deslocamento Z
    apenas nos pontos CARTESIANOS (Slots e Change Slot).
    """
    db_new = copy.deepcopy(db_original)
    
    pontos_para_ajustar = SLOT_POINTS + [POINT_CHANGE_SLOT]
    
    for nome_ponto in pontos_para_ajustar:
        if nome_ponto in db_new:
            original_pose = db_new[nome_ponto]
            # Soma o offset ao Z
            original_pose[2] += offset_z_mm
            
            # Trava de segurança
            LIMITE_INFERIOR_Z = -235.0 
            if original_pose[2] < LIMITE_INFERIOR_Z: 
                print(f"AVISO: Ponto {nome_ponto} atingiu limite inferior de Z!")
                original_pose[2] = LIMITE_INFERIOR_Z
                
    return db_new


def main(args=None):
    controller_node = None
    connect_to_robot = False
    
    try:
        choice = input("Deseja conectar ao robô? (s/n): ").strip().lower()
        connect_to_robot = (choice == 's' or choice == 'sim')
        if not connect_to_robot: 
            print("\n*** MODO SIMULAÇÃO ***\n")
            
    except KeyboardInterrupt:
        return

    # --- Inicialização ROS 2 ---
    if connect_to_robot:
        print("Iniciando ROS2...")
        rclpy.init(args=args)
        controller_node = Controller_CR3()
        
        # Inicializa o Buffer e o Listener do TF2 usando o nó do controlador
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, controller_node)

        controller_node.inicialPose()
        
        try:
            input("Enter quando atingir pose inicial...")
            print(">>> Indo para Posição de Medição...")
            controller_node.move_joints(ROBOT_POINTS_DATABASE.get(POINT_MEASURE))
            time.sleep(3) 
        except KeyboardInterrupt:
            controller_node.destroy_node()
            rclpy.shutdown()
            return
    
    if not rclpy.ok(): 
        rclpy.init(args=args)
        
    sorter = RoboticSorter()
    sorter.start_messages() 
    
    # Gravação de dados (opcional)
    LOG_FILE = "log_robot_sorter_11_Alone.csv"

    if not os.path.exists(LOG_FILE):
        with open(LOG_FILE, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp", "volume", "offset_x", "offset_y", "offset_z",
                "take_x", "take_y", "take_z", "take_rx", "take_ry", "take_rz",
                "slot1_x", "slot1_y", "slot1_z", "slot1_rx", "slot1_ry", "slot1_rz",
                "slot2_x", "slot2_y", "slot2_z", "slot2_rx", "slot2_ry", "slot2_rz",
                "slot3_x", "slot3_y", "slot3_z", "slot3_rx", "slot3_ry", "slot3_rz",
                "slot4_x", "slot4_y", "slot4_z", "slot4_rx", "slot4_ry", "slot4_rz",
                "slot1_volume", "slot2_volume", "slot3_volume", "slot4_volume",
                "change_area_height"
            ])


    # --- Loop Principal ---
    try:
        while True:
            if connect_to_robot:
                rclpy.spin_once(controller_node, timeout_sec=0.1) # Mantém o nó vivo para atualizar TFs
                
            print(f"\nEstado Atual: {sorter.get_current_state()}")
            sorter.wait_for_start_signal()
            box_size, offsets = sorter.wait_for_volume_reading()
            
            if box_size is None or offsets is None:
                print("Erro na leitura ou interrupção.")
                continue

            print(f"Dados Lidar (Metros): {offsets}")
            
            # ---------------------------------------------------------
            # 1. CÁLCULO DO DELTA Z (Agora exato e sem fatores de correção)
            # ---------------------------------------------------------
            lidar_z_atual = offsets[2]
            
            # Diferença exata em milímetros
            # Positivo = Caixa Alta (Precisa Subir para soltar)
            # Negativo = Caixa Baixa (Precisa Descer para soltar)
            delta_z_mm = (LIDAR_Z_REFERENCE_10CM - lidar_z_atual) * 1000.0
            
            print(f"--- Ajuste de Altura para Slots ---")
            print(f"   Z Ref (10cm): {LIDAR_Z_REFERENCE_10CM:.4f} m")
            print(f"   Z Atual Lido: {lidar_z_atual:.4f} m")
            print(f"   Delta Z Calculado: {delta_z_mm:.2f} mm")

            # 2. CRIA UM NOVO DB TEMPORÁRIO (Slots ajustados)
            current_points_db = apply_z_offset(ROBOT_POINTS_DATABASE, delta_z_mm)

            # ---------------------------------------------------------
            # 3. CÁLCULO DO TAKE BOX COM TF2
            # ---------------------------------------------------------
            if connect_to_robot:
                try:
                    # Cria o ponto detectado no referencial do LiDAR (em metros)
                    ponto_lidar = PointStamped()
                    ponto_lidar.header.frame_id = 'lidar_link' # Confirme se este é o nome no seu URDF
                    ponto_lidar.header.stamp = controller_node.get_clock().now().to_msg()
                    
                    ponto_lidar.point.x = offsets[0]
                    ponto_lidar.point.y = offsets[1]
                    ponto_lidar.point.z = offsets[2]

                    # Busca a relação entre a base e o lidar no instante atual
                    transform = tf_buffer.lookup_transform(
                        'base_link',   # Referencial alvo (base do Dobot)
                        'lidar_link',  # Referencial de origem (sensor)
                        rclpy.time.Time()
                    )
                    
                    # Converte magicamente o ponto para a perspectiva da base do robô
                    ponto_base = tf2_geometry_msgs.do_transform_point(ponto_lidar, transform)
                    
                    # Converte de volta para milímetros e adiciona a segurança na altura
                    new_x = ponto_base.point.x * 1000.0
                    new_y = ponto_base.point.y * 1000.0
                    new_z = (ponto_base.point.z * 1000.0) + Z_OFFSET_SAFETY 
                    
                    if new_z < 5.0: new_z = 5.0

                except Exception as e:
                    print(f"Erro no TF2. O RViz/my_dobot_descriptor está rodando? Erro: {e}")
                    continue
            else:
                # Fallback de simulação
                new_x, new_y, new_z = POINT_MEASURE_CARTESIAN[0:3]

            # Monta a pose final (mantendo rx, ry, rz originais para a garra apontar para baixo)
            calculated_pose = [new_x, new_y, new_z, POINT_MEASURE_CARTESIAN[3], POINT_MEASURE_CARTESIAN[4], POINT_MEASURE_CARTESIAN[5]]
            
            print(f"--- Relatório de Cálculo (TF2) ---")
            print(f"   Alvo Final (Take Box): {calculated_pose}")
            
            current_points_db[POINT_TAKE_BOX] = calculated_pose
            take_x, take_y, take_z, take_rx, take_ry, take_rz = calculated_pose
            
            # ---------------------------------------------
            moves = sorter.add_box(box_size)
            
            # Gravação de dados
            print("Gravando dados no log...")
            timestamp = datetime.now().isoformat()
            
            slot_poses = [current_points_db[slot] for slot in SLOT_POINTS]

            with open(LOG_FILE, mode="a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    timestamp, box_size, offsets[0], offsets[1], offsets[2],
                    take_x, take_y, take_z, take_rx, take_ry, take_rz,
                    *slot_poses[0], *slot_poses[1], *slot_poses[2], *slot_poses[3],
                    sorter.slots[0], sorter.slots[1], sorter.slots[2], sorter.slots[3],
                    sorter.change_height if hasattr(sorter, "change_height") else 0.0
                ])

            execute_move_sequence(
                moves_list=moves, 
                controller_node=controller_node, 
                points_db=current_points_db, 
                test_mode=not connect_to_robot,
                sorter_node=sorter 
            )

    except KeyboardInterrupt:
        print("\nPrograma interrompido.")

if __name__ == '__main__':
    main()