# clever-show

[English version](README.md)

Програмное обеспечение для запуска шоу дронов под управлением [Raspberry Pi](https://www.raspberrypi.org/) с пакетом COEX [Clover](https://github.com/CopterExpress/clover) и полётным контроллером с прошивкой [PX4](https://github.com/PX4/Firmware).

Создайте анимацию в [Blender](https://www.blender.org/), сконвертируйте её в полётные пути дронов, настройте дроны и запустите своё собственное шоу дронов!

## Демонстрационное видео

[![Autonomous drone show in a theater](http://img.youtube.com/vi/HdHbZFz7nR0/0.jpg)](http://www.youtube.com/watch?v=HdHbZFz7nR0)

12 дронов выступают в Электротеатре Станиславский, Москва.

## Пакет включает в себя

* [Набор ПО для дрона](drone/) с клиентским приложением для удаленного синхронизированного управления дронами и модулем экстренной посадки.
* [Серверное приложение](server/) для создания шоу и настройки дронов, анимации и музыки
* [Аддон для Blender 2.8](blender-addon/) для преобразования анимации полёта коптеров, созданной в Blender, в индивидуальные пути для каждого коптера
* [Образ для Raspberry Pi](https://github.com/CopterExpress/clever-show/releases/latest) для быстрого запуска ПО на коптере
* [Документацию](docs/ru/SUMMARY.md)

## Доступные системы позиционирования

* [Optical flow](https://clover.coex.tech/ru/optical_flow.html) - для полётов внутри помещений
* [Карта ArUco маркеров](https://clover.coex.tech/ru/aruco_map.html) - для полётов внутри помещений
* [GPS](https://clover.coex.tech/ru/gps.html) - для уличных полётов

## Быстрый старт

Начните создавать своё шоу коптеров, используя [эту инструкцию](docs/ru/start-tutorial.md)!
