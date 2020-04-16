# coding: utf-8

# -----------------------------------------------------------
# Colibri-NSK Team
#
# (C) 2020 Volokitina Veronika, Roman Manechkin
# -----------------------------------------------------------

import math
import rospy
import cv2
import numpy as np

try:
    from clover import srv
    from clover.srv import SetLEDEffect
except:
    from clever import srv
    from clever.srv import SetLEDEffect
from pyzbar import pyzbar
from cv_bridge import CvBridge
from mavros_msgs.srv import CommandBool
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

# -----------------------------------------------------------
# Объекты прокси сервисов
# -----------------------------------------------------------

rospy.init_node('flight')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
land_serv = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)


# -----------------------------------------------------------
# Все методы
# -----------------------------------------------------------

# Самый частовстречающийся элемент массива
def MostFrequent(arr):
    return max(set(arr), arr.count)


bridge = CvBridge()


# Уменьшаем размер
def Resize():
    return cv2.resize(bridge.imgmsg_to_cv2('bgr8'), (320, 240))


# Распознавание QR
def QRRecognizer(cv_image):
    arr = []
    for _ in range(3):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # Перевод в черно-белый формат
        barcodes = pyzbar.decode(gray)  # Распознавание

        for barcode in barcodes:
            (x, y, w, h) = barcode.rect  # Координаты кода
            barcodeData = barcode.data.decode("utf-8")

            # Вычисляем центр кода и переводим в топик
            xc = x + w / 2
            yc = y + h / 2
            cv_image = cv2.circle(cv_image, (xc, yc), 15, (0, 0, 0), 30)
            arr.append(barcodeData)
            image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        rospy.sleep(1)

    arr = list(filter(lambda a: a is not None, arr))
    return MostFrequent(arr)


image_pub = rospy.Publisher('~debug', Image, 1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, Resize())


# Распознавание цвета
def ColorRecognizer(cv_image):
    # Для точности цвет проверяется 3 раза, результаты проверок записываются в result[]
    result = []
    for _ in range(3):
        cv_image = cv2.resize(cv_image, (160, 120))
        height, width, _ = cv_image.shape
        height, width = height // 2, width // 2

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Границы цветов
        lower_yellow = np.array([10, 50, 80])
        upper_yellow = np.array([30, 255, 255])
        lower_red = np.array([-18, 80, 80])
        upper_red = np.array([10, 255, 255])
        lower_green = np.array([75, 20, 40])
        upper_green = np.array([100, 50, 60])

        # Кадры для кажого из диапозонов
        pic1 = cv2.inRange(hsv, lower_red, upper_red)  # Красный
        pic2 = cv2.inRange(hsv, lower_green, upper_green)  # Зеленый
        pic3 = cv2.inRange(hsv, lower_yellow, upper_yellow)  # Желтый

        # Считаем количество точек из каждого диапозона, которые находятся в центре изображения
        gre, yel, red = 0, 0, 0
        for i in range(len(pic2) // 3, len(pic2) // 3 * 2):
            for i1 in range(len(pic2[0]) // 3, len(pic2[0]) // 3 * 2):

                if pic2[i][i1] != 0:
                    gre += 1
                if pic1[i][i1] != 0:
                    red += 1
                if pic3[i][i1] != 0:
                    yel += 1

        if max(gre, yel, red) == gre and gre != 0:
            color = "green"
        elif max(gre, yel, red) == yel and yel != 0:
            color = "yellow"
        else:
            color = "red"

        result.append(color)
        rospy.sleep(0.5)
        color = MostFrequent(result)

        # Добавляем результат на пустое изображение
        image = np.zeros((height, width), np.uint8)
        cv2.putText(image, str("Result: "), (height // 4, width // 4), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
        cv2.putText(image, str(color), (height // 4, width // 4 + 20), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (255, 255, 255), 1)

        # Собираем вместе полученные изображения
        h1, w1 = pic2.shape[:2]
        h2, w2 = pic1.shape[:2]
        result1 = np.zeros((max(h1, h2), w1 + w2), np.uint8)
        result1[:h1, :w1] = pic2
        result1[:h2, w1:w1 + w2] = pic1

        h1, w1 = pic3.shape[:2]
        h2, w2 = image.shape[:2]
        result2 = np.zeros((max(h1, h2), w1 + w2), np.uint8)
        result2[:h1, :w1] = pic3
        result2[:h2, w1:w1 + w2] = image

        h1, w1 = result1.shape[:2]
        h2, w2 = result2.shape[:2]
        result = np.zeros((h1 + h2, max(w1, w2)), np.uint8)
        result[:h1, :w1] = result1
        result[h1:h1 + h2, :w2] = result2

        # Публикуем в топик
        image_pub.publish(bridge.cv2_to_imgmsg(cv2.cvtColor(result, cv2.COLOR_GRAY2BGR), 'bgr8'))
        print(color)
        return color


# Полет по маркерам
def ArucoFlight(x=0, y=0, z=0, yaw=float('nan'), speed=0.4, floor=False):
    return navigate(x, y, z, yaw, speed, 'aruco_map')


# Телеметрия по системе координат маркеров
def getTelemetryAruco():
    return get_telemetry("aruco_map")


# Взлет
def Takeoff(z):
    telem = getTelemetryAruco()
    navigate(z, 0.4, "body", True)
    rospy.sleep(2)
    ArucoFlight(telem.x, telem.y, z, 0.4, True)


# Полет до точки с ожиданием долета
def NavigationWait(x, y, z, speed=0.4, tolerance=0.13):
    ArucoFlight(x, y, z, speed)
    while not rospy.is_shutdown():
        telem = get_telemetry('navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


# Посадка
def Land(disarm=True):
    land_serv()
    rospy.sleep(5)
    arming(False)


# -----------------------------------------------------------
# Полет
# -----------------------------------------------------------

coordinates = [(0, 2.72),
               (0.72, 3.94),
               (0.72, 1.5),
               (2.88, 1.5),
               (2.16, 0.28),
               (1.44, 2.72),
               (1.44, 1.5),
               (2.16, 2.72),
               (3.6, 0.28)]

print("Init complite!")

passedCoordinates = []
otchetData = []
Takeoff(1.2)

print("Takeoff and waiting 12 seconds")

rospy.sleep(12)

for i in range(len(coordinates)):
    NavigationWait(coordinates[i][0], coordinates[i][1], 1.2)
    rospy.sleep(3)
    NavigationWait(coordinates[i][0], coordinates[i][1], 0.7)
    rospy.sleep(3)
    cv_image = Resize()
    color = ColorRecognizer(cv_image)

    if color == "red" or color == "yellow":
        print("Purple color ON")
        set_effect(128, 0, 128)
        rospy.sleep(5)
        set_effect(0, 0, 0)
        print("Purple color OFF")
        print("Сброшено")

    if color == "red":
        passedCoordinates.append(coordinates[i])
        otchetData.append([coordinates[i], "+"])
        print("red")
    elif color == "yellow":
        passedCoordinates.append(coordinates[i])
        otchetData.append([coordinates[i], "?"])
        print("yellow")
    else:
        otchetData.append([coordinates[i], "-", "Healthy"])
        print("green")
    NavigationWait(coordinates[i][0], coordinates[i][1], 1.2)
    rospy.sleep(4)

NavigationWait(0, 0, 1.2)
rospy.sleep(5)
NavigationWait(0, 0, 0.8)
Land()

print("Waiting for 2 MINS")
rospy.sleep(60)
print("Waiting for 1 MINS")
rospy.sleep(40)
print("Waiting for 20 seconds")
rospy.sleep(10)
print("Waiting for 10 seconds")
rospy.sleep(10)
print("TAKEOFF")

Takeoff(1.2)

for i in range(len(passedCoordinates)):

    NavigationWait(passedCoordinates[i][0], passedCoordinates[i][1], 1.2)
    rospy.sleep(4)
    NavigationWait(passedCoordinates[i][0], passedCoordinates[i][1], 0.8)
    rospy.sleep(3)
    cv_image = Resize()
    text = QRRecognizer(cv_image)
    print(text)
    if text == "COVID - 19":
        otchetData[coordinates.index(passedCoordinates[i])].append("COVID - 2019")
        print("Red color ON")
        set_effect(255, 0, 0)
        rospy.sleep(5)
        set_effect(0, 0, 0)
        print("Red color OFF")
    elif text == "healthy":
        otchetData[coordinates.index(passedCoordinates[i])].append("Healthy")
    else:
        otchetData[coordinates.index(passedCoordinates[i])].append("non COVID - 2019")

    NavigationWait(passedCoordinates[i][0], passedCoordinates[i][1], 1.2)
    rospy.sleep(4)

NavigationWait(0, 0, 1.2)
rospy.sleep(5)
NavigationWait(0, 0, 0.8)
rospy.sleep(5)
Land()

str1, str2, str3, str4 = "", "", "", ""

for i in range(len(otchetData)):
    str1 = str1 + str(i + 1) + ","
    str2 = str2 + "x = " + str(otchetData[i][0][0]) + " y = " + str(otchetData[i][0][1]) + ','
    str3 = str3 + str(otchetData[i][1]) + ","
    str4 = str4 + str(otchetData[i][2]) + ","

str1, str2, str3, str4 = str1[:-1], str2[:-1], str3[:-1], str4[:-1]

otchet_per = str1 + "\n" + str2 + "\n" + str3 + "\n" + str4
otchet = open("report.csv", "w")
otchet.write(otchet_per)
otchet.close()
print("Take report!!")
