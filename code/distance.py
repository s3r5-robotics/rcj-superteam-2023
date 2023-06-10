import math

import cv2 as cv
import numpy as np
import numpy.typing as npt


def map_vals(val: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class WallFilter:
    def __init__(self):
        self.hue_min = 50
        self.hue_max = 100

        self.saturation_min = 0
        self.saturation_max = 255

        self.min_value = 0
        self.max_value = 255
        self.lower = np.array([self.hue_min, self.saturation_min, self.min_value])
        self.upper = np.array([self.hue_max, self.saturation_max, self.max_value])

    def show_wall_filter_mask(self, camera_image: npt.ArrayLike):
        hsv_image = cv.cvtColor(camera_image, cv.COLOR_BGRA2BGR)
        hsv_image = cv.cvtColor(hsv_image, cv.COLOR_BGR2HSV)

        mask = cv.inRange(hsv_image, self.lower, self.upper)
        return mask

    def show_wall_filter_img_result(self, camera_image: npt.ArrayLike):
        image_result = cv.bitwise_and(camera_image, camera_image, mask=self.show_wall_filter_mask(camera_image))
        return image_result


class DistanceCalculator:
    def __init__(self, height: int, width: int):
        self.height: int = height
        self.width: int = width

        self.filters = WallFilter()

    def __distance_measuring(self, binary_image: npt.ArrayLike) -> int:
        counter = 0
        for i in range(0, self.height):
            if binary_image[i][math.floor(self.width / 2)] == 255:
                counter += 1
        return counter

    def pixel_height(self, camera_image: npt.ArrayLike) -> int:
        filters = self.filters.show_wall_filter_mask(camera_image)
        pixels_height = self.__distance_measuring(filters)

        return pixels_height

    def distance_calculation(self, camera_image: npt.ArrayLike) -> float:
        filters = self.filters.show_wall_filter_mask(camera_image)
        pixels_height = self.__distance_measuring(filters)  # ToDo: can improve
        print(f"ph {pixels_height}")
        distance_base = 37.11126
        pixels_height_base = 52
        distance = (pixels_height_base * distance_base) / pixels_height
        print(f"d {distance}")

        return distance
