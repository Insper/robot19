# coding: utf-8

import sys
import math
import pygame

from pf import Particle
from config import *
import game_utils
import inspercles

import projeto_pf # Lembre-se de que seu código está na projeto_pf somente


def nonzero(list_):
    """
        Checks if a list has a value other then zero
    """
    for a in list_:
        if a!=0:
            return True
    return False


class Window:
    """
        Main window of the application
    """
    def __init__(self, on_update=None, background_file=None, robot=None):
        '''
        Args:
            background_file(str): Background filename
            robot(Particle): Particle representing actual robot
            particles(list[Particle]): Simulation particles
        '''
        self.on_update = on_update

        # Initialize view
        pygame.init()
        if background_file is None:
            background_file = BACKGROUND_FILE
        self.background = pygame.image.load(background_file)
        self.size = self.background.get_size()
        self.screen = pygame.display.set_mode(self.size)

        # Initialize simulation components
        self.robot_speed = [0, 0] # [LINEAR_x, LINEAR_y, ANGULAR]
        self.robot = robot
        if self.robot is None:
            self.robot = Particle(self.size[0]/2, self.size[1]/2)
        projeto_pf.particulas = projeto_pf.cria_particulas()

        self.leituras_robot = inspercles.nb_lidar(self.robot, projeto_pf.angles)


        self.ofb = pygame.Surface(self.size)
        self.ofb.set_colorkey((0,0,0))

    def run(self):
        '''
        Game loop. 
        '''
        while True:
            self.on_events(pygame.event.get())
            if nonzero(self.robot_speed):
                self.robot.move_relative(self.robot_speed)
                self.leituras_robot = inspercles.nb_lidar(self.robot, projeto_pf.angles)

                # move particles
                projeto_pf.move_particulas(projeto_pf.particulas, self.robot_speed)
                # draw lasers

                  # Atualiza probabilidade e posicoes
                projeto_pf.leituras_laser_evidencias(projeto_pf.robot, projeto_pf.particulas)

                # Reamostra as particulas
                projeto_pf.particulas = projeto_pf.reamostrar(projeto_pf.particulas)

            if self.on_update is not None:
                self.on_update(self.robot, projeto_pf.particulas, self.robot_speed)

            self.draw()

    def on_events(self, events):
        '''
        Handle user generated events.
        '''
        for event in events:
            print("event: ", event)
            print("speed: ", self.robot_speed)
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key in SPEED_DELTAS:
                    delta = SPEED_DELTAS[event.key]
                    self.robot_speed[delta[0]] += delta[1]
                    print(event.key)
            elif event.type == pygame.KEYUP:
                if event.key in SPEED_DELTAS:
                    delta = SPEED_DELTAS[event.key]
                    self.robot_speed[delta[0]] -= delta[1]

    def draw(self):
        '''
        Draw everything (background, particles, and robot).
        '''
        self.screen.fill(BLACK)
        self.screen.blit(self.background, self.background.get_rect())
        self.ofb.fill(BLACK)


        # Draw maze - debugging reasons
        # Descomente para ver um efeito psicodélico
        #game_utils.draw_maze(self.ofb)


        # Draw particles
        for particle in projeto_pf.particulas:
            self.draw_particle(particle)

        if self.leituras_robot:
            game_utils.draw_laser_readings(self.ofb, self.robot, self.leituras_robot)

        self.draw_robot()



        self.ofb.convert()         # Is this really necessary?


        # Draws offscreen buffer to front
        self.screen.blit(self.ofb, (0,0))

        pygame.display.flip()

    def draw_robot(self):
        # Position
        r = ROBOT_RADIUS
        center = (int(self.robot.x), int(self.robot.y))

        # Draw
        pygame.draw.circle(self.ofb, ROBOT_COLOR, center, r)
        self.draw_particle(self.robot, ROBOT_ARROW_COLOR)

    def draw_particle(self, particle, color=ARROW_COLOR):
        arrow = pygame.Surface((2*ARROW_WIDTH, 2*ARROW_WIDTH), pygame.SRCALPHA, 32)
        arrow = arrow.convert_alpha()

        sp = (ARROW_WIDTH, ARROW_WIDTH)
        ep = (2*ARROW_WIDTH, ARROW_WIDTH)
        ep1 = (2*ARROW_WIDTH - ARROW_END_LEN, ARROW_WIDTH + ARROW_HEIGHT)
        ep2 = (2*ARROW_WIDTH - ARROW_END_LEN, ARROW_WIDTH - ARROW_HEIGHT)

        pygame.draw.line(arrow, color, sp, ep, ARROW_THICK)
        pygame.draw.line(arrow, color, ep, ep1, ARROW_THICK)
        pygame.draw.line(arrow, color, ep, ep2, ARROW_THICK)

        arrow = pygame.transform.rotate(arrow, -math.degrees(particle.theta))
        rect = arrow.get_rect()
        self.ofb.blit(arrow, (int(particle.x - rect.width / 2), int(particle.y - rect.height / 2)))


count= 0
skip = 30

if __name__ == '__main__':
    from random import random


    print("Se \n \tpython window.py \n não funcionar, tente: \n \tpythonw window.py \n especialmente no Python 3")

    def on_update(robot, particles, robot_speed):
        global count
        count += 1
        if not count%skip:
            print(robot.x, robot.y, robot.theta)

    Window(on_update=on_update).run()
