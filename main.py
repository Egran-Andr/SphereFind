import cv2
import numpy as np

#Тестирование метода поиска кругов на изображении

# загрузка изображения
img = cv2.imread('img.png')
output = img.copy()

# Изменение и выравнивание цифровой палитры
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
gray = cv2.GaussianBlur(gray, (9, 9), 2)

circles = cv2.HoughCircles(
    gray,
    cv2.HOUGH_GRADIENT,
    dp=1.2,
    minDist=30,
    param1=130,
    param2=30,
    minRadius=20,
    maxRadius=90
)

# Если найден круг
if circles is not None:
    circles = np.round(circles[0, :]).astype("int")
    for (x, y, r) in circles:
        # Отрисовка краев
        cv2.circle(output, (x, y), r, (0, 255, 0), 0, 5)
        # Отрисовка центра
        cv2.circle(output, (x, y), 3, (0, 0, 255), -1)
        print("Detected center (x, y):", x, y)
        print("Detected radius:", r)

# Результат
cv2.imshow('Detected Ball', output)
cv2.waitKey(0)
cv2.destroyAllWindows()
