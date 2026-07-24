import pygame
import sys

# -------------------------
# Initialize Pygame
# -------------------------
pygame.init()

# Small window to capture keyboard input
screen = pygame.display.set_mode((300, 200))
pygame.display.set_caption("Mecanum Robot Control")

# -------------------------
# Movement Functions
# -------------------------
def forward():
    print("Forward")

def backward():
    print("Backward")

def strafe_left():
    print("Strafe Left")

def strafe_right():
    print("Strafe Right")

def rotate_left():
    print("Rotate Left")

def rotate_right():
    print("Rotate Right")

def stop():
    print("Stop")

# -------------------------
# Main Loop
# -------------------------
running = True

while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()

    if keys[pygame.K_w]:
        forward()

    elif keys[pygame.K_s]:
        backward()

    elif keys[pygame.K_a]:
        strafe_left()

    elif keys[pygame.K_d]:
        strafe_right()

    elif keys[pygame.K_q]:
        rotate_left()

    elif keys[pygame.K_e]:
        rotate_right()

    else:
        stop()

pygame.quit()
sys.exit()
