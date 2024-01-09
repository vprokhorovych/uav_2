from __future__ import print_function
import time
import argparse
import numpy as np

from math import degrees

from dronekit import connect, Vehicle, VehicleMode
from geographiclib.geodesic import Geodesic

TS = 0.5  # time step; кожні TS секунд ми фіксуємо поточний стан дрона та оновлюємо керування

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


def change_channel_temporarily(overrides, channel, delta_rc, t=TS):
    """Змінює значення каналу `channel` на `delta_rc` одиниць, чекає `t` секунд, а потім, якщо
    `t < TS` скидає канал до 'trim' значення (1500) і чекає, поки не спливе `TS`-проміжок. Якщо ж
     `t >= TS`, то просто змінюємо значення каналу. `overrides` - це `vehicle.channels.overrides`.
    """

    overrides[channel] = 1500 + delta_rc
    if t >= TS:
        time.sleep(TS)
    else:
        time.sleep(t)
        overrides[channel] = 1500
        time.sleep(TS - t)
    

def get_pitch(dist, dist_error, debug=True):
    """Емпірична функція для встановлення pitch angle дрона. `dist` - відстань у метрах до точки
    призначення, чим вона менша, тим менший нахил (і швидкість) дрона. `dist_error` - допустима
    похибка у метрах, якщо коптер знаходиться не далі, аніж `dist_error` метрів від кінцевої точки,
    вважається, що він прибув (і тоді подальший рух припиняється).

    Зауважимо, що дрон рухається лише вперед - рух назад чи вбік не реалізований.
    """

    if dist < dist_error:
        delta = 0  # delta - скільки треба додати до базового значення pitch
    elif dist < 3:
        delta = -50 # Із незрозумілих причин чомусь при значеннях |delta| < 50
                    # коптер зупиняється, а не помалу рухається вперед чи назад
    elif dist < 12:
        delta = -70
    elif dist < 50:
        delta = -120
    else:
        delta = -500

    if debug:
        print(f'Curr dist:  {dist:.4f}') #, "\tdelta: ", delta, "\tres: ", res)

    return delta


def get_yaw(delta_azim, delta_error, debug=True):
    """Емпірична функція для встановлення yaw angle дрона. `delta_azim` - різниця в градусах між поточним
    та бажаним азимутом. `delta_error` - допустима додатня похибка у градусах, якщо поточний азимут
    відрізняється від бажаного (за модулем), не більше, аніж на `delta_error`, то ми yaw angle припиняємо змінювати.
    Чим більше значення `delta_azim`, тим швидше дрон розвертається.

    Повертає пару `(delta, t)`, де `delta` - на скільки змінюємо відповідний канал, а `t` - час, протягом якого
    ця зміна триватиме (`t` не може перевищувати `TS`)
    """
    # коефіцієнт зменшення TS
    if -2 < delta_azim < 2:
        c = 0.25
    elif -4 < delta_azim < 4:
        c = 0.5
    else:
        c = 1

    t = TS * c

    if -180 <= delta_azim <= -50:
        delta = -120
    elif -50 < delta_azim <= -15:
        delta = -65
    elif -15 < delta_azim <= -delta_error:
        delta = -45
    elif -delta_error < delta_azim < delta_error:
        delta = 0
    elif delta_error <= delta_azim < 15:
        delta = 50
    elif 15 <= delta_azim < 50:
        delta = 75
    elif 50 <= delta_azim <= 180:
        delta = 120
    else:
        raise ValueError(f'Delta azim value {delta} is incorrect')
    
    if debug:
        print(f"\tDelta azim:  {delta_azim:.4f}, delta:  {delta}, time  {t:.4f}")

    return delta, t


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
    
    `delta_azim = "azim - curr_azim"` (докладніше див. `get_delta_azim`)
    """

    c_lat, c_lon = vehicle.location.global_frame.lat, vehicle.location.global_frame.lon  # поточні координати

    dist, azim =  calculate_dist_azimuth(c_lat, c_lon, lat, lon)  # відстань і азимут до кінцевої точки відносно поточної
    azim = norm_azim(azim)  # нормування азимута
    curr_azim = get_curr_azim(vehicle)
    delta_azim = get_delta_azim(curr_azim, azim)

    return dist, azim, curr_azim, delta_azim


def rotate(vehicle: Vehicle, lat, lon, azim_error=1):
    """Розвертає коптер так, аби він дивився в точку `(lat, lon)` (ці координати не мають задавати
    поточне положення). `azim_error` - допустима похибка в градусах.
    
    Ідея дуже проста - із певною частотою змінюємо значення rc-канала, відповідального за азимут, допоки
    не досягнемо мети. Проблема - ми не можемо ані часто знімати показники стану дрона (наприклад, його
    поточні координати - якщо намагатися це робити 4 рази на секунду, то замість чотирьох різних значень
    `[x_1, x_2, x_3, x_4]` отримаємо лише 2 різні значення `[x_1, x_1, x_2, x_2]`), ані давати малі зусилля 
    на rc-канали (приміром, мінімальна кутова швидкість за годинниковою стрілкою скаладає приблизно 
    3.8 градуса/с, якій відповідають значення yaw-каналу 1550, 1551, ..., 1559, а при 1560, ..., 1569
    ця швидкість вже сягатиме десь 7.7 градуса/с. На щастя, ми можемо "неперервно" керувати часом, протягом
    якого зміна значення каналу житиме - приміром, можна збільшити pitch angle на 0.15 с. Звісно, від інерції
    нікуди не втечеш.
    """

    overrides = vehicle.channels.overrides

    _, azim, _, delta_azim = get_info(vehicle, lat, lon)  # нам треба лише два значення (одне з них - лише для консолі)
    print("Desired azim: ", azim)

    while abs(delta_azim) >= azim_error:  # інерцією можна знехтувати - зауваження ті ж, що й у функції move_to
        dy, t = get_yaw(delta_azim, azim_error)
        change_channel_temporarily(overrides, '4', dy, t)

        *_, delta_azim = get_info(vehicle, lat, lon)  # дрон все одно трохи зміщується,
                                                      # тому оновлення координат не зашкодить
        
    *_, curr_azim, delta_azim = get_info(vehicle, lat, lon)
    print(f"Rotated; curr azim:  {curr_azim:.4f}, desired azim:  {azim:.4f}, azim error:  {delta_azim:.4f}")

    # Зупинити розвертання
    overrides['4'] = 1500


def rotate_yaw(vehicle: Vehicle, azim, azim_error=1):
    """Повертає коптер, поки його азимут не збіжиться з `azim` з точністю до `azim_error`. Функціонально
    цей метод еквівалентний методу `rotate`, зауваження також ті самі"""

    overrides = vehicle.channels.overrides

    print("Desired azim: ", azim)

    delta_azim = get_vehicle_delta_azim(vehicle, azim)

    while abs(delta_azim) >= azim_error:  # інерцією можна знехтувати - зауваження ті ж, що й у функції move_to
        dy, t = get_yaw(delta_azim, azim_error)
        change_channel_temporarily(overrides, '4', dy, t)
        delta_azim = get_vehicle_delta_azim(vehicle, azim)
        
    curr_azim = get_curr_azim(vehicle)
    delta_azim = get_vehicle_delta_azim(vehicle, azim)

    print(f"Rotated; curr azim:  {curr_azim:.4f}, desired azim:  {azim:.4f}, azim error:  {delta_azim:.4f}")

    # Зупинити розвертання
    overrides['4'] = 1500


def get_delta_azim(az1, az2):
    """Вертає найменший кут, на який треба повернути азимут `az1`, щоб він збігся з азимутом `az2` (обидва
    набувають значень з [0, 360)). Повертає значення з півінтервала (-180, 180], де додатні числа
    означають рух за годинниковою стрілкою
    """

    delta_azim = min(x for x in (az2 - az1, az2 - az1 + 360) if x > 0)
    if delta_azim > 180:
        delta_azim -= 360

    return delta_azim

def get_vehicle_delta_azim(vehicle: Vehicle, azim):
    curr_azim = get_curr_azim(vehicle)

    return get_delta_azim(curr_azim, azim)


# Відображення (число, що додаємо до 1500 yaw-каналу -> кутова швидкість коптера в град/с.),
# знайдене експериментальним шляхом

YAW_DCT = {
    50: 3.8, 60: 7.7, 70: 11.5, 80: 15.4, 90: 19.1, 100: 23, 110: 26.7, 120: 30.6,
    130: 34.4, 140: 42.5, 150: 45.9, 160: 50, 170: 54.9, 180: 57.5, 190: 61.6, 200: 65, 150: 45,
    -45 : -3.8, -55: -7.7, -65: -11.6, -75: -15.5, -85: -19.3, -95: -26.5, -105: -30.5,
    -115: -34.5, -125: -34.5, -135: -42.3, -145: -46.2, -155: -50, -165: -53.6, -175: -57.6,
    -185: -61.3, -195: -69.1, -205: -72.8
}

def get_yaw_move(dist, delta_azim, azim_error=1, debug=True):
    """Повертає пару `(dy, t)`, де `dy` - на скільки змінюємо значення yaw-каналу, `t` - час,
    протягом якого змінене на `dy` значення залишається таким (ці значення підуть в 
    `change_channel_temporarily`, див. її документацію щодо детальнішого опису цих параметрів).
    Ця функція призначена для вирівнювання азимута дрона під час руху. Зусилля треба прикладати
    короткотермінові, інакше за `TS == 0.5` секунд навіть при найменшій зміні `dy = 50` та малому
    відхилення 1.5 градуса коптер не стане на потрібний азимут, що з'єднує початкову й кінцеву точки,
    а перелетить за нього, і знову доведеться азимут рівняти. Також треба акуратно керувати, коли
    коптер наближається до кінцевї точки - там маленькі лінійні зміщення породжують спричиняють великі 
    азимутові розбіжності.
    
    Алгоритм емпіричний - маючи відхилення `E`, шукаємо найменшу кутову швидкість з `YAW_DICT`, яка дозволить
    це відхилення подолати за `t <= TS` секунд, причому цю швидкість ми застосовуємо протягом часу меншого
    за `t` (визначається змінною `coef`), щоби по інерції потрапити в `azim_error`-окіл потрібного азимута.
    Біля кінцевої точки ми штучно за допомогою коефіцієнта `c` збільшуємо цю мінімальну швидкість, оскільки
    там азимут треба змінювати швидко.
    """

    if abs(delta_azim) < azim_error:
        dy = 0
        t = TS
    else:
        candidates = {}

        # c використовуємо, щоби штучно збільшити delta_azim - якщо
        # нам потрібна вища швидкість
        if dist < 4:
            c = 1.5
        elif dist < 12:
            c = 1.4
        elif dist < 20:
            c = 1.3
        else:
            c = 1
        
        for dy, v in YAW_DCT.items():

            t = c * delta_azim / v

            if t > 0:
                candidates[t] = dy
            if t > 0 and t <= TS:
                if dist < 20:
                    coef = 0.9
                elif dist < 70:
                    coef = 0.8
                else:
                    coef = 0.4
                t *= coef

                break
        else:
            dy = candidates[min(candidates)]
            t = TS

    if debug:
        print(f'\tDelta azim:  {delta_azim:.4f},\tdy:  {dy},\ttime:  {t:.4f}')

    return dy, t


def move_to(vehicle: Vehicle, lat, lon, dist_error=0.5, azim_error=1):
    """Основна функція, яка здійснює рух до точки з координатами `(lat, lon)` (вважаємо, що 
    AltHold уже встановлений). `dist_error` - похибка, якщо до кіцевої точки відстань менша,
    то зупиняємося. `azim_error` - допустима похибка для азимута. Алгоритм руху дуже простий - 
    рухатися вперед, регулярно виправлюючи поточні азимут і лінійну швидкість.
    
    Коптер рухається виключно вперед, назад чи вліво-вправо - ні.
    """

    overrides = vehicle.channels.overrides

    dist, *_, delta_azim = get_info(vehicle, lat, lon)
    print("Init dist: ", dist)

    deltas = []

    while dist >= dist_error:  # Інерцією можна знехтувати, бо get_pitch дає майже незмінене значення
                               # pitch, коли ми майже прибули; хоча, звісно, якщо значення dist_error мале
                               # то інерцію таки треба брати до уваги. Наприклад, можна збільшити частоту
                               # оновлення rc-каналів (можливо, треба змінити умову "зупинки", приміром, чекати
                               # кілька секунд, і якщо коптер не відлетів далеко - тільки тоді анулювати нахил)
        
        deltas.append(delta_azim)

        # зміну pitch встановлюємо вручну - вона в нас діє постійно на відміну від зміни yaw
        overrides['2'] = 1500 + get_pitch(dist, dist_error)

        dy, t = get_yaw_move(dist, delta_azim, azim_error)
        change_channel_temporarily(overrides, '4', dy, t)

        # оновити дані, щоби скоригувати нахил і поворот
        dist, *_, delta_azim = get_info(vehicle, lat, lon)

    print("\nArrived; curr dist: ", dist)  # азимут не виводжу, бо нам важливо дістатися точки, 
                                           # а який там вже буде азимут - байдуже
    # Астанавітєсь!
    overrides['2'] = 1500
    overrides['4'] = 1500

    return deltas



# ---------------------------- Головний скрипт --------------------------------------
# здійнятися на вказану висоту в метрах
height = 100

arm_and_takeoff(height)

# перейти в режим AltHold
vehicle.mode = VehicleMode("ALT_HOLD")
vehicle.channels.overrides['3'] = 1500  # без цього дрон заоре носом
time.sleep(1)


# B = (50.449222, 30.459837)
# B = 50.448203, 30.458096
B = 50.443326, 30.448078
rotate(vehicle, *B, 0.5)
time.sleep(5)

# уперед!
deltas = move_to(vehicle, *B)
print('\nYaw deltas stat:')
print("avg:  ", np.average(deltas))
print("min:  ", np.amin(deltas))
print("max:  ", np.amax(deltas))
print("med:  ", np.median(deltas))
print("std:  ", np.std(deltas))

time.sleep(2)
print("\nArrived; setting yaw to 350\n")

# встановити азимут в 350 градусів
rotate_yaw(vehicle, 350, 0.5)
time.sleep(5)

if sitl:
    print("Shutting down the simulator")
    sitl.stop()