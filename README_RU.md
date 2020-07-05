# clever-show

[English version](README.md)

Програмное обеспечение для запуска шоу дронов под управлением Raspberry Pi с пакетом COEX [Clever](https://github.com/CopterExpress/clever) и полётного контроллера с прошивкой PX4.

Создайте анимацию в Blender, сконвертируйте её в полётные пути дронов, настройте дроны и запустите своё собственное шоу дронов!

[![Build Status](https://travis-ci.org/CopterExpress/clever-show.svg?branch=master)](https://travis-ci.org/CopterExpress/clever-show)

## Демонстрационное видео

[![Autonomous drone show in a theater](http://img.youtube.com/vi/HdHbZFz7nR0/0.jpg)](http://www.youtube.com/watch?v=HdHbZFz7nR0)

12 дронов выступают в Электротеатре Станиславский, Москва.

## Пакет включает в себя

* [Набор ПО для дрона](https://github.com/CopterExpress/clever-show/tree/master/Drone) с клиентским приложением для удаленного синхронизированного управления дронами и модулем экстренной посадки.
* [Серверное приложение](https://github.com/CopterExpress/clever-show/tree/master/Server) для создания шоу и настройки дронов, анимации и музыки
* [Аддон для Blender 2.8](https://github.com/CopterExpress/clever-show/tree/master/blender-addon) для преобразования анимации полёта коптеров, созданной в Blender, в файлы полётов для каждого коптера
* [Образ для Raspberry Pi](https://github.com/CopterExpress/clever-show/releases/latest) для быстрого запуска ПО на коптере

## Документация

Инструкция по запуску ПО находится [здесь](docs/ru/start-tutorial.md).

Подробная документация расположена в папке [docs](https://github.com/CopterExpress/clever-show/tree/master/docs).
