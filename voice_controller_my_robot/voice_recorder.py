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
            self.recorder()

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
        
        self.recognizer(file_path)
        
    
    def recognizer(self, file_path):
        # Initialisation du reconnaisseur de parole
        r = sr.Recognizer()

        # Spécifiez le chemin vers votre fichier audio
        audio_file = file_path

        # Utilisation du fichier audio au lieu du microphone
        with sr.AudioFile(audio_file) as source:
            print("Reading audio file...")
            audio_data = r.record(source)  # Lire l'intégralité du fichier audio
            print("End reading!")

        # Reconnaissance vocale
        try:
            result = r.recognize_google(audio_data, language="fr-FR")
            # Pour une reconnaissance de la parole en anglais
            # result = r.recognize_google(audio_data, language="en-EN")
            print("Vous avez dit : ", result)
            self.commande_interpretor(result)
        except sr.UnknownValueError:
            print("Google Speech Recognition n'a pas pcmdu comprendre l'audio.")
        except sr.RequestError as e:
            print(f"Erreur de requête avec Google Speech Recognition; {e}")
            
    
    def commande_interpretor(self, commande):
        # Expression régulière pour identifier les commandes de type "va à la salle X" ou "va en salle Y"
        pattern = re.compile(r"va\s+(?:à\s+la\s+salle?\s*(\d+)|en\s+(?:la\s+)?salle?\s*(\d+))", re.IGNORECASE)
        match = pattern.match(commande.strip())

        if match:
            # Extraire le numéro de la salle
            numero_de_salle = match.group(1) or match.group(2)
            print(f"Commande valide : {commande} -> Numéro de la salle : {numero_de_salle}")
            msg = Int16()
            msg.data = int(numero_de_salle)
            self.publisher_.publish(msg)
        else:
            print(f"Commande invalide : {commande}")
        

        
def main(args=None):
    rclpy.init(args=args)
    node = VoiceRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
