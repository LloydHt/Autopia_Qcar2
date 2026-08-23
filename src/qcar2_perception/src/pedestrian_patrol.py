#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class PedestrianPatrol(Node):
    def __init__(self):
        super().__init__('pedestrian_patrol_node')
        
        # Le topic correspond au namespace que tu as mis dans le plugin du SDF
        # Change "/anymal_test/cmd_vel" par le bon si besoin
        self.publisher_ = self.create_publisher(Twist, '/anymal_test/cmd_vel', 10)
        
        # Paramètres de la traversée (Open-Loop Control)
        self.crossing_speed = -0.25 # Vitesse de marche (m/s)
        self.crossing_time = 4.0    # Temps pour traverser (secondes) -> Distance = Vitesse * Temps
        self.wait_time = 3.0        # Temps d'attente sur le trottoir

        # États : 0 = Attente (Départ), 1 = Traverse, 2 = Attente (Arrivée), 3 = Retour
        self.state = 0
        self.state_start_time = time.time()
        
        # Boucle de contrôle à 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("🤖 Cerveau du piéton activé : Mode Patrouille sur passage piéton.")

    def timer_callback(self):
        msg = Twist()
        current_time = time.time()
        elapsed = current_time - self.state_start_time

        if self.state == 0:
            # Attend sur le trottoir initial
            if elapsed > self.wait_time:
                self.change_state(1)
                
        elif self.state == 1:
            # Traverse la route
            msg.linear.y = self.crossing_speed
            if elapsed > self.crossing_time:
                self.change_state(2)
                
        elif self.state == 2:
            # Attend sur le trottoir d'en face
            if elapsed > self.wait_time:
                self.change_state(3)
                
        elif self.state == 3:
            # Fait demi-tour (marche en arrière/inverse pour revenir)
            msg.linear.y = -self.crossing_speed
            if elapsed > self.crossing_time:
                self.change_state(0) # Recommence la boucle

        self.publisher_.publish(msg)

    def change_state(self, new_state):
        self.state = new_state
        self.state_start_time = time.time()
        
        states_text = ["Attente (Trottoir 1)", "Traversée (Aller)", "Attente (Trottoir 2)", "Traversée (Retour)"]
        self.get_logger().info(f"🚶 Changement d'état : {states_text[self.state]}")


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()