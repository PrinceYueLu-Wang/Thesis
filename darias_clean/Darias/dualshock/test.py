import pygame
pygame.init()


joystick=pygame.joystick.Joystick(0)

joystick.init()

clock = pygame.time.Clock()
color = 0

running = True

button_idx=14
while running:

    for event in pygame.event.get():

        if event.type == pygame.JOYBUTTONDOWN:

            print(event.button)

            if event.button == button_idx:

                print("Pressed!!")

        if event.type == pygame.JOYBUTTONUP:


            if event.button == button_idx:

                print("Released!!")


        
