from __future__ import print_function
import time
import argparse
import numpy as np

from math import degrees

from dronekit import connect, Vehicle, VehicleMode
from geographiclib.geodesic import Geodesic
import matplotlib.pyplot as plt

# Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(tgt_alt):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(tgt_alt)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # reaching required altitude
        if tgt_alt * 1.02 >= vehicle.location.global_relative_frame.alt >= tgt_alt * 0.98:
            print("Reached target altitude")
            break
        time.sleep(1)


def calculate_dist_azimuth(lat1, lon1, lat2, lon2):
    """Обчислює відстань та азимут між точками з координатами `(lat1, lon1), (lat2, lon2)`"""

    geod = Geodesic.WGS84
    g = geod.Inverse(lat1, lon1, lat2, lon2)
    return g['s12'], g['azi1']  # відстань та азимут


def get_pitch(dist, dist_error, print_msg=True):
    """Емпірична функція для встановлення pitch angle дрона. `dist` - відстань у метрах до точки
    призначення, чим вона менша, тим менший нахил (і швидкість) дрона. `dist_error` - допустима
    похибка у метрах, якщо коптер знаходиться не далі, аніж `dist_error` метрів від кінцевої точки,
    вважається, що він прибув (і тоді подальший рух припиняється).

    Зауважимо, що дрон рухається лише вперед - рух назад чи вбік не реалізований.
    """

    if dist < dist_error:
        delta = 0  # delta - скільки треба додати до базового значення pitch
    elif dist < 20:
        delta = 55  # Із незрозумілих причин чомусь при значеннях delta < 50
                    # коптер зупиняється, а не помалу рухається вперед чи назад
    elif dist < 40:
        delta = 70
    elif dist < 40:
        delta = 120
    else:
        delta = 1000

    res = 1500 - delta  # 1500 - базове значення pitch, яке означає відсутність нахилу дрона

    if print_msg:
        print("Curr dist: ", dist) #, "\tdelta: ", delta, "\tres: ", res)

    return res


def get_yaw(delta_azim, delta_error):
    """Емпірична функція для встановлення yaw angle дрона. `delta_azim` - різниця в градусах між поточним
    та бажаним азимутом. `delta_error` - допустима додатня похибка у градусах, якщо поточний азимут
    відрізняється від бажаного (за модулем), не більше, аніж на `delta_error`, то ми yaw angle припиняємо змінювати.
    Чим більше значення `delta_azim`, тим швидше дрон розвертається.
    """

    abs_delta_azim = abs(delta_azim)
    sgn = 1 if delta_azim > 0 else (-1 if delta_azim < 0 else 0)  # потрібно для визначення напрямку розвороту
    
    if abs_delta_azim < delta_error:
        delta = 0
    elif abs_delta_azim < 15:
        delta = 47 if sgn == 1 else 52  # Аналогічно до get_pitch  при значеннях delta < 45
                                        # коптер припиняє розвертатися; більше того чомусь для
                                        # повороту за год. стр. треба трохи більший delta
    elif abs_delta_azim < 80:
        delta = 65
    else:
        delta = 100

    res = 1500 - sgn * delta  # 1500 - базове значення yaw, яке означає відсутність розвертання дрона
    print(f"Delta azim: {delta_azim}, res: {res}")
    return res


def norm_azim(azim):
    """Нормує азимут - нам потрібна область значень [0, 360) (обчислення за годинниковою стрілкою).
    Азимут `azim` має набувати значень (-180, 180]
    """

    return azim + (360 if -180 <= azim < 0 else 0)


def get_curr_azim(vehicle):
    """Повертає нормалізований поточний азимут коптера в градусах"""

    return norm_azim(degrees(vehicle.attitude.yaw))


def get_info(vehicle: Vehicle, lat, lon):
    """Повертає кортеж `(dist, azim, curr_azim, delta_azim)`, де

    `dist` - відстань від коптера до точки з координатами `(lat, lon)`,
    
    `azim` - азимут вектора з початком у lat, lon координатах дрона і кінцем в `(lat, lon)`,
    
    `curr_azim` - поточний азимут коптера
    
    `delta_azim = curr_azim - azim`
    """

    c_lat, c_lon = vehicle.location.global_frame.lat, vehicle.location.global_frame.lon  # поточні координати

    dist, azim =  calculate_dist_azimuth(c_lat, c_lon, lat, lon)  # відстань і азимут до кінцевої точки відносно поточної
    azim = norm_azim(azim)  # нормування азимута
    curr_azim = norm_azim(degrees(vehicle.attitude.yaw))  # нормування поточного азимута
    delta_azim = curr_azim - azim

    return dist, azim, curr_azim, delta_azim


def move_to(vehicle: Vehicle, lat, lon, dist_error=0.5):
    """Основна функція, яка здійснює рух до точки з координатами `(lat, lon)` (вважаємо, що 
    AltHold уже встановлений). `dist_error` - похибка, якщо до кіцевої точки відстань менша,
    то зупиняємося. Алгоритм руху дуже простий - рухатися вперед, регулярно виправлюючи поточні
    азимут і лінійну швидкість.
    
    Коптер рухається виключно вперед, назад чи вліво-вправо - ні.
    """

    overrides = vehicle.channels.overrides
    time_step = 0.25  # частота оновлення rc-каналів

    dist, azim, curr_azim, delta_azim = get_info(vehicle, lat, lon)
    print("Init dist: ", dist)

    while dist >= dist_error:  # Інерцією можна знехтувати, бо get_pitch дає майже незмінене значення
                               # pitch, коли ми майже прибули; хоча, звісно, якщо значення dist_error мале
                               # то інерцію таки треба брати до уваги. Наприклад, можна збільшити частоту
                               # оновлення rc-каналів (можливо, треба змінити умову "зупинки", приміром, чекати
                               # кілька секунд, і якщо коптер не відлетів далеко - тільки тоді анулювати нахил)
        
        # встановлюємо "швидкості" нахилу й повороту на time_step секунд
        overrides['2'] = get_pitch(dist, dist_error)
        overrides['4'] = get_yaw(curr_azim - azim, 1)
        time.sleep(time_step)

        # оновити дані, щоби скоригувати нахил і поворот
        dist, azim, curr_azim, delta_azim = get_info(vehicle, lat, lon)

        # Якщо ми близько до місця призначення, то дуже важливо, аби коптер був спрямованим на кінцеву точку -
        # бо коптер рухається лише вперед, а в цих обставинах малі зміни в просторі спричиняють великі зміни
        # азимута (наприклад, якщо дрон пролетів кінцеву точку, то йому доведеться розвертатися на 180 градусів),
        # тому коптер зупиняється і встановлює правильний кут повороту (з точністю до 1.5 градуса) на кінцеву точку,
        # і лише потім продовжує рух. Ця зміна поведінки помітна - в околі кінцевої точки дрон рухається "обережно",
        # малими кроками і частіше виставляє правильний азимут, аніж власне зміщується
        while abs(delta_azim) > 1.5 and 5 + dist_error > dist >= dist_error:
            overrides['2'] = get_pitch(0, dist_error, False)  # False потрібне, інакше в консоль виведеться "Curr dist: 0",
                                                              # що хибно
            time.sleep(time_step)
            overrides['4'] = get_yaw(curr_azim - azim, 1)
            time.sleep(time_step)

            dist, azim, curr_azim, delta_azim = get_info(vehicle, lat, lon)

    print("\nArrived; curr dist: ", dist)  # азимут не виводжу, бо нам важливо дістатися точки, 
                                           # а який там вже буде азимут - байдуже
    # Астанавітєсь!
    overrides['2'] = get_pitch(0, dist_error, False)
    overrides['4'] = get_yaw(0, 1)  # замість одиниці можна поставити будь-яке додатнє значення
    

def rotate(vehicle: Vehicle, lat, lon, azim_error=1):
    """Розвертає коптер так, аби він дивився в точку `(lat, lon)` (ці координати не мають задавати
    поточне положення). `azim_error` - допустима похибка в градусах.
    
    Ідея дуже проста - із певною частотою змінюємо значення rc-канала, відповідального за азимут, допоки
    не досягнемо мети.
    """

    overrides = vehicle.channels.overrides
    time_step = 0.25

    _, azim, _, delta_azim = get_info(vehicle, lat, lon)  # нам треба лише два значення (одне з них - лише для консолі)
    print("Desired azim: ", azim)

    while abs(delta_azim) >= azim_error:  # інерцією можна знехтувати - зауваження ті ж, що й у функції move_to 
        overrides['4'] = get_yaw(delta_azim, azim_error)
        time.sleep(time_step)

        *_, delta_azim = get_info(vehicle, lat, lon)  # дрон все одно трохи зміщується,
                                                      # тому оновлення координат не зашкодить
        
    *_, curr_azim, delta_azim = get_info(vehicle, lat, lon)
    print("Rotated; curr azim: ", curr_azim, "\tazim error: ", delta_azim)

    # Зупинити розвертання
    overrides['4'] = get_yaw(0, azim_error)


def rotate_yaw(vehicle: Vehicle, azim, azim_error=1):
    """Повертає коптер, поки його азимут не збіжиться з `azim` з точністю до `azim_error`. Функціонально
    цей метод еквівалентний методу `rotate`"""

    overrides = vehicle.channels.overrides
    time_step = 0.25

    print("Desired azim: ", azim)

    curr_azim = get_curr_azim(vehicle)
    delta_azim = curr_azim - azim

    while abs(delta_azim) >= azim_error:
        overrides['4'] = get_yaw(delta_azim, azim_error)
        time.sleep(time_step)

        curr_azim = get_curr_azim(vehicle)
        delta_azim = curr_azim - azim

    print("Rotated; curr azim: ", curr_azim, "\tazim error: ", delta_azim)

    # Зупинити розвертання
    overrides['4'] = get_yaw(0, azim_error)


# ---------------------------- Головний скрипт --------------------------------------
# здійнятися на вказану висоту в метрах
height = 10
arm_and_takeoff(height)

# перейти в режим AltHold
vehicle.mode = VehicleMode("ALT_HOLD")
vehicle.channels.overrides['3'] = 1500  # без цього дрон заоре носом
time.sleep(1)

time_interval = 10  # seconds
step = 0.5
# rc_deltas = [25, 50, 100, 150, 200, 225]#, 250, 300, 350, 400, 450, 500] # -500, 0]
# rc_deltas = [50, 60, 70, 80, 90, 100]#, 250, 300, 350, 400, 450, 500] # -500, 0]
# rc_deltas = [-40, -41, -42, -43, -44, -45, -46, -47, -48, -49, -50]#, 250, 300, 350, 400, 450, 500] # -500, 0]
rc_deltas = [-45, -55, -65, -75, -85, -95, -105, -115, -125, -135, -145, -155, -165, -175, -185, -195, -205]#, 250, 300, 350, 400, 450, 500] # -500, 0]
# rc_deltas = [50, 75, 100, 125, 150]#, 250, 300, 350, 400, 450, 500] # -500, 0]

x = np.linspace(0, len(rc_deltas) * time_interval, len(rc_deltas) * int(time_interval / step), endpoint=False)
yaws = np.zeros(len(x))
xx = np.zeros_like(yaws)

idx = 0
for delta in rc_deltas:
    start = time.time()
    print(f'Setting yaw delta to\t{delta}; time = \t{start}')
    vehicle.channels.overrides['4'] = 1500 + delta
    while time.time() - start < time_interval:
    
        yaws[idx] = get_curr_azim(vehicle)  # normalised in degrees
        idx += 1
        time.sleep(step)

print(f'Simuliation finished; idx = {idx}, \tyaws len = {len(yaws)}')
vehicle.channels.overrides['4'] = 1500

delta_yaws = yaws[1:] - yaws[:-1]

for i, delta in enumerate(delta_yaws):
    if abs(delta) > 0.6 * step and delta * rc_deltas[0] < 0:  # 1.5 - 356.7
        delta_yaws[i] += 360 if rc_deltas[0] > 0 else -360

yaws_v = delta_yaws / step  # velocities

yaws_a = (yaws_v[1:] - yaws_v[:-1]) / step  # acceleration

with open('test.npy', 'wb') as f:
    np.save(f, x)
    np.save(f, yaws)
    np.save(f, delta_yaws)
    np.save(f, yaws_v)
    np.save(f, yaws_a)

fig, (ax0, ax1, ax2) = plt.subplots(nrows=3)

ax0.plot(x, yaws, 'b')
ax0.set_title("Yaw")
ax0.legend()

ax1.plot(x[:-1], yaws_v, 'g')
ax1.set_title('Yaw velocity')
ax1.legend()

ax2.plot(x[:-2], yaws_a, 'r')
ax2.set_title("Yaw acceleration")
ax2.legend()


plt.tight_layout()
plt.show()
