import pygame
pygame.init()


joystick=pygame.joystick.Joystick(0)

joystick.init()

clock = pygame.time.Clock()
color = 0

running = True

button_idx=14
# while running:

#     for event in pygame.event.get():

#         if event.type == pygame.JOYBUTTONDOWN:

#             print(event.button)

#             if event.button == button_idx:

#                 print("Pressed!!")

#         if event.type == pygame.JOYBUTTONUP:


#             if event.button == button_idx:

#                 print("Released!!")

button_dict={'2':"up",
             "0":"down",
             "3":"left",
             "1":"right",
             "10":"reset"}


while running:

    for event in pygame.event.get():

        if event.type == pygame.JOYBUTTONDOWN:

            idx=str(event.button)

            if button_dict.__contains__(idx):

                idx=button_dict[idx]

                if idx == "up":
                    print("up is pressed!")
                elif idx == "down":
                    print("down is pressed!")
                elif idx == "left":
                    print("left is pressed!")
                elif idx == "right":
                    print("right is pressed!")
                elif idx == "reset":
                    print("reset is pressed!")
            else:

                print("not in the list!")



        
