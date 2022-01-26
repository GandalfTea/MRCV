
from trial import *
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from multiprocessing import Process, Queue

def renderPoint(coords, size, color):
    glEnable(GL_POINT_SMOOTH)
    glPointSize(size)

    glBegin(GL_POINTS)
    glColor3d(color[0], color[1], color[2])
    x = coords[0]
    y = coords[1]
    z = coords[2]
    glVertex3d(x, y, z)
    glEnd()


def render(queue):
    button_down = False
    pygame.init()
    display = (1000, 800)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)


    gluPerspective(45, (display[0]/display[1]), 0.1, 5000.0)
    glTranslatef(0.0, -50.0, -200)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    glTranslatef(-0.5, 0, 0)
                if event.key == pygame.K_RIGHT:
                    glTranslatef(0.5, 0, 0)

                if event.key == pygame.K_UP:
                    glTranslatef(0, 1, 0)
                if event.key == pygame.K_DOWN:
                    glTranslatef(0, -1, 0)
            if event.type == pygame.MOUSEMOTION:
                if(button_down):
                    glRotatef(event.rel[1], 1.0, 0, 0)
                    glRotatef(event.rel[0], 0, 1.0, 0)
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:
                    glTranslatef(0, 0, 100.0)
                if event.button == 5:
                    glTranslatef(0, 0, -100.0)
        x = glGetDoublev(GL_MODELVIEW_MATRIX)
        camera_x = x[3][0]
        camera_y = x[3][1]
        camera_z = x[3][2]

        for event in pygame.mouse.get_pressed():
            if pygame.mouse.get_pressed()[0] == 1:
                button_down = True
            elif pygame.mouse.get_pressed()[0] == 0:
                button_down = False


        #glRotate(1.5, 5, 5, 5)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

        #Cube()
        pts = queue.get()
        for f in pts[0]:
            renderPoint(f[:3, 3], 5, (0,0, 1))
        for d in pts[1]:
            renderPoint(d, 1, (1,1,1))


        pygame.display.flip()
        #pygame.time.wait(10)


class SlamRender:
    def __init__(self, arg):
        POINTS_TO_RENDER = Queue()

        slam = Slam()
        video = Process(target=slam.run, args=(arg, (POINTS_TO_RENDER)))
        plot = Process(target=render, args=((POINTS_TO_RENDER), ))

        video.start()
        plot.start()





