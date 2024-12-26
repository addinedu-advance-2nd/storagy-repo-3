import pygame
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

music_file1 = "/home/storagy/Desktop/PINKLAB/tts/후방좌측_수저.mp3" 
music_file2 = "/home/storagy/Desktop/PINKLAB/tts/전방좌측_바구니.mp3"
music_file3 = "/home/storagy/Desktop/PINKLAB/tts/전방우측_사무용품.mp3"
music_file4 = "/home/storagy/Desktop/PINKLAB/tts/계산대.mp3"
music_file5 = "/home/storagy/Desktop/PINKLAB/tts/사용자인식불가_뇌절.mp3"
freq = 16000    
bitsize = -16   
channels = 1   
buffer = 2048   

class ServiceTts(Node):
    def __init__(self):
        super().__init__('tts_service')
        self.srv = self.create_service(AddTwoInts, '/tts_status', self.callback)

        pygame.mixer.init(freq, bitsize, channels, buffer)
        self.get_logger().info('TTS service server activated.')

    def callback(self, request, response):
            response.sum = request.a + request.b

            if response.sum == 2:
                pygame.mixer.music.load(music_file1)
                pygame.mixer.music.play()
                clock = pygame.time.Clock()
                while pygame.mixer.music.get_busy():
                    clock.tick(30)
  
            elif response.sum == 3:
                pygame.mixer.music.load(music_file2)
                pygame.mixer.music.play()
                clock = pygame.time.Clock()
                while pygame.mixer.music.get_busy():
                    clock.tick(30)
            
            elif response.sum == 4:
                pygame.mixer.music.load(music_file3)
                pygame.mixer.music.play()
                clock = pygame.time.Clock()
                while pygame.mixer.music.get_busy():
                    clock.tick(30)

            elif response.sum == 5:
            	
                pygame.mixer.music.load(music_file4)
                pygame.mixer.music.play()
                clock = pygame.time.Clock()
                while pygame.mixer.music.get_busy():
                    clock.tick(30)

            elif response.sum == 6:
                pygame.mixer.music.load(music_file5)
                pygame.mixer.music.play()
                clock = pygame.time.Clock()
                while pygame.mixer.music.get_busy():
                    clock.tick(30)
            return response


def main():
    rclpy.init()
    srv = ServiceTts()
    rclpy.spin(srv)
    rclpy.shutdown()
    pygame.mixer.quit()  

if __name__ == '__main__':
    main()
