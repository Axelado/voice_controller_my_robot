# importer les bibliothèques nécessaires
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int16
from ament_index_python.packages import get_package_share_directory
import os

import sounddevice as sd
import wavio
import speech_recognition as sr
import re

package_name = "voice_controller_my_robot"
# définir le node
class VoiceRecorder(Node):
    def __init__(self):
        super().__init__('voice_recorder')
        self.publisher_ = self.create_publisher(Int16, 'room_number', 10)
        self.subscription = self.create_subscription(
            String,
            'start_talking',  
            self.listener_callback,
            10)
        self.subscription 

    def listener_callback(self, msg):
        if msg.data == "1":
            file_path = self.recorder()
            text = self.recognizer(file_path)
            if text != None:
                room_number = self.commande_interpretor(text)
                if room_number != None:
                    self.publish_room_number(room_number)
                
            

    def recorder(self):
        # Paramètres de l'enregistrement
        duration = 4  # Durée de l'enregistrement en secondes
        sample_rate = 44100  # Fréquence d'échantillonnage en Hz

        self.get_logger().info("Enregistrement en cours...")
        audio_data = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=2, dtype='int16')
        sd.wait()  # Attendre la fin de l'enregistrement
        self.get_logger().info("Enregistrement terminé.")

        # Sauvegarde de l'audio dans un fichier
        directory_path = os.path.join(get_package_share_directory(package_name), 'records')
        os.makedirs(directory_path, exist_ok=True)
        
        filename = "enregistrement.wav"
        file_path = os.path.join(directory_path, filename)
        
        wavio.write(file_path, audio_data, sample_rate, sampwidth=2)
        self.get_logger().info(f"L'enregistrement a été sauvegardé sous le nom de '{file_path}'.")
        
        return file_path
        
        
        
    
    def recognizer(self, file_path):
        # Initialisation du reconnaisseur de parole
        r = sr.Recognizer()

        audio_file = file_path
        try:
            # Utilisation du fichier audio au lieu du microphone
            with sr.AudioFile(audio_file) as source:
                self.get_logger().info("Reading audio file...")
                audio_data = r.record(source)  # Lire l'intégralité du fichier audio
                self.get_logger().info("End reading!")

            # Reconnaissance vocale
            try:
                result = r.recognize_google(audio_data, language="fr-FR")
                # Pour une reconnaissance de la parole en anglais
                # result = r.recognize_google(audio_data, language="en-EN")
                self.get_logger().info(f"Vous avez dit : , {result}")
            except sr.UnknownValueError:
                self.get_logger().info("Google Speech Recognition n'a pas pu comprendre l'audio.")
                result = None
            except sr.RequestError as e:
                self.get_logger().info(f"Erreur de requête avec Google Speech Recognition; {e}")
                result = None
        except FileNotFoundError:
            self.get_logger().info(f"Le fichier audio {audio_file} n'a pas été trouvé.")
            result = None
        except Exception as e:
            self.get_logger().info(f"Une erreur s'est produite lors de la lecture du fichier audio : {e}")
            result = None
            
        if os.path.exists(audio_file):
            try:
                os.remove(audio_file)
                self.get_logger().info(f"Le fichier audio {audio_file} a été supprimé.")
            except Exception as e:
                self.get_logger().info(f"Une erreur s'est produite lors de la suppression du fichier audio : {e}")
        return result
            
    
    def commande_interpretor(self, commande):
        # Expression régulière pour identifier les commandes de type "va à la salle X" ou "va en salle Y"
        pattern = re.compile(r"va\s+(?:à\s+la\s+salle?\s*(\d+)|en\s+(?:la\s+)?salle?\s*(\d+))", re.IGNORECASE)
        match = pattern.match(commande.strip())

        if match:
            # Extraire le numéro de la salle
            room_number = match.group(1) or match.group(2)
            self.get_logger().info(f"Commande valide : {commande} -> Numéro de la salle : {room_number}")
            return int(room_number)
        
        else:
            self.get_logger().info(f"Commande invalide : {commande}")
            return None

    def publish_room_number(self, room_number):
        msg = Int16()
        msg.data = int(room_number)
        self.publisher_.publish(msg)
        
        

        
def main(args=None):
    rclpy.init(args=args)
    node = VoiceRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
