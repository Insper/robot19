# coding: utf-8
from random import randint, choice
import time
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import math
import random
from pf import Particle
import rayline
import cv2
from intersection.intersection import find_intersections
from intersection.segment import Segment
from occupancy_field_numpy import OccupancyField
from time import time


initial_pose = []

particle_size = 7

lines = None

occupied_thresh = 0.2
free_thresh = 0.9

robot_radius = 10

width = 775

height = 746

back_color = "black"
colors     = ['red', 'green', 'cyan', 'yellow']

color_image = cv2.imread("sparse_obstacles.png")
pil_image = color_image
np_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

# Aplica transformada de distância
occupancy_field = OccupancyField(color_image)

lidar_map = None

retorno_lidar_robo = np.copy(np_image)


EPS = 1e-8

def canny_lines(img):
    """
        Retorna todos os segmentos de linha contidos numa imagem
    """
    np_image = img
    canny = cv2.Canny(np_image, occupied_thresh*255, free_thresh*255)
    #kernel = np.ones((5,5), np.uint8)
    #canny = cv2.dilate(canny, kernel, iterations=1)
    linhas = np.array([])
    lines = cv2.HoughLinesP(image=canny,rho=0.02,theta=np.pi/1000, threshold=25,lines=linhas, minLineLength=minLineLength,maxLineGap=3)
    return lines

lines = None


map_name = 'sparse_obstacles'
img = cv2.imread(map_name + '.png')
with open(map_name + '_lines.txt', 'r') as f:
    segments = np.array([[float(j) for j in line.split()] for line in f])

minLineLength = img.shape[1]/45


lines = segments


if lines is None:
    lines = canny_lines(img)


#print("\t\tLINES:\n")s
#print(lines)

def make_vecs(xs, ys):
    '''
    Creates 2D numpy array from two 1D arrays.

    Args:
        xs (nx1 numpy array): x coordinates array
        ys (nx1 numpy array): y coordinates array

    Returns:
        vecs (nx2 numpy array): combined array
    '''
    return np.hstack((vert(xs), vert(ys)))


def make_3d_vecs(xs, ys):
    '''
    Creates 3D numpy array from two 2D arrays.

    Args:
        xs (nxm numpy array): x coordinates array
        ys (nxm numpy array): y coordinates array

    Returns:
        vecs (nxmx2 numpy array): combined array
    '''
    return np.dstack((xs, ys))


def compute_norms(vecs):
    '''
    Compute norm of 2D vectors.

    Args:
        vecs (nx2 numpy array): array with 2D vectors

    Returns:
        norms (nx1 numpy array): vector norms
    '''
    return np.sqrt(vecs[:,0]*vecs[:,0] + vecs[:,1]*vecs[:,1])


def vert(v):
    '''
    Transform v into vertical array.

    Args:
        v (numpy array): array to be transformed

    Returns:
        vertical (nx1 numpy array): transformed array
    '''
    return v.reshape((-1, 1))


def hor(v):
    '''
    Transform v into horizontal array.

    Args:
        v (numpy array): array to be transformed

    Returns:
        horizontal (1xn numpy array): transformed array
    '''
    return v.reshape((1, -1))


def are_parallel(seg_directs, directions):
    '''
    Checks if seg_directs and directions are parallel.

    Args:
        seg_directs (nx2 numpy array): segment directions computed by subtracting one end point
            by the other directions (mx2 numpy array): direction vectors

    Returns:
        parallel (nxm numpy array): parallel[i,j] is True if segment direction i is parallel
            to direction j
    '''
    norms = compute_norms(seg_directs)
    seg_directs = seg_directs / make_vecs(norms, norms)
    dx = hor(directions[:,0])
    dy = hor(directions[:,1])
    return (np.abs((vert(seg_directs[:,0])-dx) < EPS) & (np.abs(vert(seg_directs[:,1])-dy) < EPS)) | \
           (np.abs((vert(seg_directs[:,0])+dx) < EPS) & (np.abs(vert(seg_directs[:,1])+dy) < EPS))


def compute_intersections(pt, directions, segments):
    '''
    Compute all intersection points in each direction.

    Args:
        origin (tuple): x, y coordinates
        directions (mx2 numpy array): normalized directions
        segments (nx4 numpy array): segments are represented by (x1, y1, x2, y2),
            where (x1, y1) and (x2, y2) are the end points

    Returns:
        valid (nxm numpy array): valid[i,j] is True if intersection between segment
            i and direction j is valid
        intersections (nxmx2 numpy array): intersections[i,j,:] is the intersection point
            between segment i and direction j
    '''
    n = segments.shape[0]
    angles_n = directions.shape[0]
    px, py = pt
    ctheta = hor(directions[:,0])
    stheta = hor(directions[:,1])
    x1, y1, x2, y2 = vert(segments[:, 0]), vert(segments[:, 1]), vert(segments[:, 2]), vert(segments[:, 3])
    p1 = make_3d_vecs(np.repeat(x1, angles_n, axis=1), np.repeat(y1, angles_n, axis=1))
    p2 = make_3d_vecs(np.repeat(x2, angles_n, axis=1), np.repeat(y2, angles_n, axis=1))
    denom = (vert(x2 - x1) * stheta + vert(y1 - y2) * ctheta)
    s = (vert(px - x1) * stheta + vert(y1 - py) * ctheta) / denom
    r = (y1 + s * vert(y2 - y1) - py) / stheta
    use_x = (abs(ctheta) > abs(stheta)).flatten()
    r[:,use_x] = ((x1 + s * vert(x2 - x1) - px) / ctheta)[:,use_x]
    intersections = make_3d_vecs(px + r * ctheta, py + r * stheta)
    valid = np.repeat(True, n * angles_n).reshape(n, angles_n)

    # Check for parallel cases
    seg_directs = make_vecs(segments[:,2] - segments[:,0], segments[:,3] - segments[:,1])
    parallel_directs = are_parallel(seg_directs, directions)
    between1 = make_vecs(x1 - px, y1 - py)
    between2 = make_vecs(x2 - px, y2 - py)
    norms_between1 = compute_norms(between1)
    norms_between2 = compute_norms(between2)
    # p1 = pt
    in_p1 = vert(norms_between1 < EPS)
    idx = parallel_directs & in_p1
    intersections[idx, :] = p1[idx, :]
    # parallel
    not_collinear = (~are_parallel(between1, directions)) & (~in_p1)
    valid[parallel_directs & not_collinear] = False
    # pt = p1 or pt = p2
    collinear_p1 = (~not_collinear) & vert(norms_between1 < norms_between2)
    collinear_p2 = (~not_collinear) & vert(norms_between1 >= norms_between2)
    intersections[parallel_directs & collinear_p1,:] = p1[parallel_directs & collinear_p1,:]
    intersections[parallel_directs & collinear_p2,:] = p2[parallel_directs & collinear_p2,:]

    # Check validity
    not_valid = (np.abs(denom) < EPS) | (r < -EPS) | (s < -EPS) | (s > 1+EPS)
    valid[(~parallel_directs) & not_valid] = False

    return valid, intersections





def convert_to_figure(xy_theta):
    """
        Converts a xy_theta to screen coordinates
    """
    pass


def nb_draw_map(mapa_numpy, particles = None, initial_position=False, pose=False, robot=False):
    """
        particles - um conjunto de partículas definidas como objetos do tipo partícula

        initial_position - cor para desenhar a posição inicial do robo

        pose - pose do robo

        robot - booleano que determina se o robô é desenhado como um círculo ou não
    """
    fig, ax = plt.subplots(figsize=(10,10))
    ax.set(xlim=[0, width], ylim=[0, height]) # Or use "ax.axis([x0,x1,y0,y1])"

    fig.canvas.draw()

    #def update():

    plt.imshow(mapa_numpy, cmap='Greys_r')
    #if initial_position:
    #    draw_initial_pose(initial_pose,ax)
    if particles:
        nb_draw_particle_cloud(particles, ax)
    if pose:
        nb_draw_arrow(pose[0], pose[1], pose[2], ax, color='g', width=2, headwidth=6, headlength=6)
    if robot:
        nb_draw_robot(pose, ax, radius=robot_radius)

    return ax # Retornamos o contexto grafico caso queiram fazer algo depois


def draw_initial_pose(pose_xytheta, ax):
    """
        Metodo que desenha a pose inicial
        pose - um array que contem x, y e theta da pose inicial
        ax - um objeto do matplotlib
    """
    x = pose_xytheta[0]
    y = pose_xytheta[1]
    theta = pose_xytheta[2]
    l = 15
    #end_x = x + deltax
    #end_y = y + deltay
    nb_draw_arrow(x, y, theta, ax, l=l, color='r', width=2, headwidth=6, headlength=6)

def nb_draw_arrow(x, y, theta, ax, l = 15, color='y', headwidth=3.0, headlength=3, width=0.001):
    """
        Desenha uma seta na posição x, y com um ângulo theta
        ax é o contexto gráfico

    """
    deltax = l*math.cos(theta)
    deltay = l*math.sin(theta)
    ax.arrow(x, y, deltax, deltay, head_width=headwidth, head_length=headlength, fc=color,  ec=color, width=width)

def nb_draw_particle_cloud(particles, ax):
    """
        Desenha o particle cloud
        particles - uma lista de objetos Particle
        ax - eixo
    """
    for p in particles:
        nb_draw_arrow(p.x, p.y, p.theta, ax, particle_size, color='b')

def normalize_particles(particle_cloud):
    #
    #global particle_cloud
    w_sum = 0
    for p in particle_cloud:
        w_sum+=p.w
    for p in particle_cloud:
        p.normalize(w_sum)

def update_robot_pose(particle_cloud, W):
    """
        O objetivo deste item é fornecer uma estimativa da pose do robo

        Pode-se escolher como isto é feito.

        Por exemplo:
            Usar a média de todas as partículas
            Usar as partículas mais provaveis
    """
    robot_pose = [0, 0, 0]
    return robot_pose

def nb_initialize_particle_cloud(xy_theta=None):
    """ Initialize the particle cloud.
        Arguments
        xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                  particle cloud around.  """
    if xy_theta == None:
        #xy_theta = convert_pose_to_xy_and_theta(.odom_pose.pose)
        pass
    global particle_cloud
    # TODO create particles
    particle_cloud = nb_create_particles(initial_pose)

    normalize_particles(particle_cloud)
    update_robot_pose(particle_cloud, np.ones(len(particle_cloud)))
    return particle_cloud

def nb_create_particles(pose, var_x = 50, var_y = 50, var_theta = math.pi/3, num=30):
    """
        Cria num particulas
        situadas no intervalo x - var_x a x + var_x, y - var_x at'e y + var_y e theta - var_theta a theta + var_theta
    """
    particle_cloud = []
    s = pose
    for i in range(num):
        x = random.uniform(s[0] - var_x, s[0] + var_x)
        y = random.uniform(s[1] - var_y, s[1] + var_y)
        theta = random.uniform(s[2] - var_theta, s[2] + var_theta)
        p = Particle(x, y, theta, w=1.0) # A prob. w vai ser normalizada depois
        particle_cloud.append(p)
    return particle_cloud

def nb_draw_robot(position, ax, radius=10):
    """
        Desenha um círculo com uma seta para se passar pelo robô
    """
    from matplotlib.patches import Circle
    circle = Circle((position[0], position[1]), radius, facecolor='none',
                    edgecolor=(0.0, 0.8, 0.2), linewidth=2, alpha=0.7)
    ax.add_patch(circle)

def nb_create_ros_map(numpy_image):
    """
        Este notebook nao usa o service GetMap, portanto
        precisamos usar a imagem que foi lida e criar um OccupancyGrid
    """
    grid = OccupancyGrid()
    grid.info.resolution = 1
    w = numpy_image.shape[0]
    h = numpy_image.shape[1]
    grid.info.width = w
    grid.info.height = h
    image_data = []
    for i in range(numpy_image.size):
        cell = 1.0 - (numpy_image[i//w][i%w]/255.0)
        if cell < 0.005:
            cell = 0
        image_data.append(cell)

    print("Occurences of zero",image_data.count(0))
    grid.data = image_data
    return grid

def nb_interp(min_a, max_a, a, dst_min, dst_max):
    """
        Funcao de interpolacao generica.
        min_a
        max_a
        a - valor na faixa de origem
        dst_min, dst_max - a faixa de destino
    """
    return dst_min + ((a - min_a)/(max_a - min_a))*(dst_max - dst_min)

def nb_cria_occupancy_field_image(occupancy_field, numpy_image):
    occupancy_image = numpy_image.copy() # todo: extrair do occupancy_grid no futuro
    max_dist = max(occupancy_field.closest_occ.values())
    min_dist = min(occupancy_field.closest_occ.values())
    for i in range(occupancy_image.shape[1]):
        for j in range(occupancy_image.shape[0]):
            occupancy_image[i][j] = int(nb_interp(min_dist, max_dist, occupancy_field.get_closest_obstacle_distance(j,i), 0, 255))
    return occupancy_image




def nb_outside_image(x, y, img):
    if x >= img.shape[1] or x < 0:
        return True
    if y >= img.shape[0] or y < 0:
        return True

def nb_found_obstacle(x, y, x0, y0, img):
    gray_value = 1.0 - img[x][y]/255.0
    if gray_value > free_thresh and gray_value < occupied_thresh:
        return math.sqrt( (x0 - x)**2 + (y0 - y)**2 )



def nb_find_discrete_line_versor(xa, ya, angle):
    """
        Encontra a direção para a qual o sensor laser do robô no ângulo angle aponta
    """
    m = math.tan(angle)
    delta = 50.0 # arbitrario
    xd = xa + delta*math.cos(angle)
    yd = ya + delta*math.sin(angle)
    deltay = yd - ya
    deltax = xd - xa
    v = [deltax, deltay]
    length = math.sqrt(deltax**2 + deltay**2)
    versor = [deltax/delta, deltay/delta]
    # Um pouco ineficiente mas 'garante' que nao pularemos celulas
    for i in range(len(versor)):
        versor[i]*=0.9
    return versor



def nb_simulate_lidar(robot_pose, angles, img, retorno = None, output_image=True):
    """
        Simula a leitura `real` do LIDAR supondo que o robot esteja na robot_pose e com sensores nos angulos angles

        Nao e' necessario fazer isso em seu projeto

        retorna uma lista de pontos de intersecao ou -1 se o sensor nao ler nada naquele angulo

    """
    a = angles.copy()
    theta = 2 # para ficar mais intuitivo

    #robot_pose[theta] = angle_normalize(robot_pose[theta])

    lidar_results = {}


    result_img = None

    if output_image:
        if retorno == None:
            result_img = np.zeros(img.shape)
        else:
            result_img = retorno

        result_img.fill(255) # Deixamos tudo branco


    x0 = robot_pose[0]
    y0 = robot_pose[1]

    # Se o robô simulado (que pode ser uma partícula) já estiver fora da imagem, retornamos zero
    if nb_outside_image(int(x0), int(y0), img):
        for a in angles:
            lidar_results[a] = 0

        return lidar_results, result_img



    for angulo in a:
        # Faz o angulo ser relativo ao robo
        ang = robot_pose[theta]+angulo
        #print("Angle ", ang)
        xa, ya = x0, y0
        x = xa
        y = ya
        vers = nb_find_discrete_line_versor(xa, ya, ang)
        #print("vers ", ang, "  " , vers)

        while (True):
            if output_image:
                result_img[int(y), int(x)] = 0 # Marcamos o raio na imagem y,x porque numpy e' linha, coluna
            if nb_outside_image(int(x), int(y), img):
                # A imagem acabou, nao achamos nada
                lidar_results[angulo] = 0
                print("Outside at ",x ,"  ",y, "  for angle ", ang)
                break
            dist = nb_found_obstacle(int(y), int(x), y0, x0, img)
            if dist > -1:
                # Achamos alguma coisa
                lidar_results[angulo] = dist
                #print("Hit for ",x,  "  ",y, "  for angle ", ang)
                break
            # Keep going if none of the "ifs" has been triggered
            x += vers[0]
            y += vers[1]

            if y > img.shape[0] or x > img.shape[1]:
                break


    return lidar_results, result_img





def intersecao_mais_proxima(ray_origin, ray_direction, lines):
    """
        Dentre as intereseçoes, acha a mais próxima
    """
    intersecoes = intersecao_linhas(ray_origin, ray_direction, lines)
    x = ray_origin[0]
    y = ray_origin[1]
    dists = np.sqrt(np.power(intersecoes[:,0]-x,2)+ np.power(intersecoes[:,1]-y,2))
    minimo = np.min(dists)
    i, = np.where( dists==minimo )
    i = i[0]
    p_int = (intersecoes[i,0] , intersecoes[i, 1])
    return dists[i], p_int



def intersecao_linhas(ray_origin, ray_direction, lines):
    """
        Acha todas as intersecoes entre o raio e as linhas dentro de um conjunto de linhas
    """
    results = []
    for i in range(lines.shape[0]):
        p1 = (lines[i][0][0], lines[i][0][1])
        p2 = (lines[i][0][2], lines[i][0][3])
        inter = rayline.lineRayIntersectionPoint(ray_origin, ray_direction, p1, p2)
        #print("p1 ", p1)
        #print("p2 ", p2)
        #print("inter v2 ", inter[0])
        results.append(inter[0])
    return np.array(results, dtype=float)

def make_directions(particle, angles):
    """
        Returns a list of normalized direction vectors
        for all the angles of the robot's lasers
        in map coordinate frame
    """
    absolute_angles = [particle.theta + angle for angle in angles]
    normed = np.array([(math.cos(a), math.sin(a)) for a in absolute_angles]).reshape((-1, 2))
    return normed




def nb_lidar_old(particle, angles):
    global lidar_map
    leituras, temp = nb_simulate_lidar_fast(particle.pose(), angles, np_image, output_image=False)
    return leituras

def nb_lidar(particle, angles, lines = lines, fast=False, occupancy_field=occupancy_field):
    directions = make_directions(particle, angles)
    if fast:
        sensor_radius = 5
        sensors = (directions * sensor_radius).astype(np.uint8)
        dists = occupancy_field.closest_occ[sensors[:,0], sensors[:,1]]
        readings = dict(zip(angles, dists))
        return readings
    else:
        origin = (particle.x, particle.y)
        interpoints = closest_intersections(origin, directions, lines)
        dists = []
        for p in interpoints:
            if p is None:
                dist = float('inf')
            else:
                dist = math.sqrt((p[0]-origin[0])**2 + (p[1] - origin[1])**2)
            dists.append(dist)
        readings= dict(zip(angles, dists))
        return readings


def closest_intersections(origin, directions, segments):
    '''
    Find closest intersection point in each direction.

    Args:
        origin (tuple): x, y coordinates
        directions (mx2 numpy array): normalized directions
        segments (nx4 numpy array): segments are represented by (x1, y1, x2, y2),
            where (x1, y1) and (x2, y2) are the end points

    Returns:
        closest (list): the returned list has one element for each direction.
            The element is the closest intersection point in that direction.
            If there is not intersection in that direction, the element is None.
    '''
    valid, intersections = compute_intersections(origin, directions, segments)
    closest = []
    for i in range(valid.shape[1]):
        valid_intersections = intersections[valid[:,i],i,:]
        if valid_intersections.shape[0] == 0:
            closest.append(None)
            continue
        dif = valid_intersections - origin
        dsq = dif[:,0]*dif[:,0] + dif[:,1]*dif[:,1]
        closest.append(valid_intersections[np.argmin(dsq),:])
    return closest


def nb_simulate_lidar_desenha(particle, angles):
    """
        Computes the closes intersection in evey given direction
        And traces laser results into imagem_saida
    """
    # inefficient, since we're redoing this
    results = nb_lidar(particle, angles, lines)
    angles_result = sorted(results.keys())

    imagem_saida = np.copy(color_image)

    for r in angles_result:
        cv2.line(imagem_saida, (int(particle.x), int(particle.y)), (int(particle.x + math.cos(r+particle.theta)*results[r]), int(particle.y + math.sin(r+particle.theta)*results[r])), (0,0,0),1, lineType=cv2.LINE_AA)

    return results, imagem_saida




def nb_simulate_lidar_fast(robot_pose, angles, img, retorno = None, output_image=True):
    """
        Simula a leitura `real` do LIDAR supondo que o robot esteja na robot_pose e com sensores nos angulos angles

        Nao e' necessario fazer isso em seu projeto

        retorna uma lista de pontos de intersecao ou -1 se o sensor nao ler nada naquele angulo

    """
    a = angles.copy()
    theta = 2 # para ficar mais intuitivo

    #robot_pose[theta] = angle_normalize(robot_pose[theta])

    lidar_results = {}


    global lines

    result_img = None

    if output_image:
        if retorno is None:
            result_img = np.zeros(img.shape)
        else:
            result_img = retorno

        result_img.fill(255) # Deixamos tudo branco

    if lines is None:
        lines = canny_lines(img)


    x0 = robot_pose[0]
    y0 = robot_pose[1]

    # Se o robô simulado (que pode ser uma partícula) já estiver fora da imagem, retornamos zero
    if nb_outside_image(int(x0), int(y0), img):
        for a in angles:
            lidar_results[a] = 0

        return lidar_results, result_img


    # Faz o angulo ser relativo ao robo
    a = [angulo + robot_pose[theta] for angulo in a]
    segments = []
    for i in range(lines.shape[0]):
        p1 = (lines[i][0], lines[i][1])
        p2 = (lines[i][2], lines[i][3])
        segments.append(Segment(p1, p2, ref=(x0, y0)))
    inters, visible_segments = find_intersections([x0, y0], segments, a)
    for angulo in a:
        inter = inters[angulo]
        if inter is None:
            lidar_results[angulo] = -1
        else:
            lidar_results[angulo] = inter.distance

            if output_image == True:
                ponto = inter.intersection
                cv2.line(result_img,(int(x0),int(y0)),(int(ponto[0]),int(ponto[1])),(0,0,0),1)

    return lidar_results, result_img






