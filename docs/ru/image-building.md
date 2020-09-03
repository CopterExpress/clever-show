# Сборка модифицированного образа

Иногда возникает необходимость собрать образ с настройками коптера, отличными от релизной версии образа. Есть несколько способов это сделать.

## Подготовка к сборке

Установите [docker](https://www.docker.com):

```bash
sudo apt install docker.io
```

## Локальная сборка с изменением настроек Клевера

* Поместите папки с файлами настроек Клевера (`launch`, `map` и `camera_info`) в [папку](../../builder/clever-config) `builder/clever-config` в директории с исходным кодом clever-show. 
  * Все файлы из папки `launch` будут скопированы в директорию `/home/pi/catkin_ws/src/clever/clever/launch` в собранном образе.
  * Все файлы из папки `map` будут скопированы в директорию `/home/pi/catkin_ws/src/clever/aruco_pose/map` в собранном образе.
  * Все файлы из папки `camera_info` будут скопированы в директорию `/home/pi/catkin_ws/src/clever/clever/camera_info` в собранном образе.
* Соберите свой образ с помощью docker:

```bash
cd source-dir
sudo docker run --privileged -it --rm -v /dev:/dev -v $(pwd):/mnt goldarte/img-tool:v0.5
```

## Ручная настройка образа

* Разархивируйте файл со скачанным образом, перейдите в директорию с этим образом, и войдите в консоль сборщика образа с помощью команды:

```bash
cd image-dir
sudo docker run --privileged -it --rm -v /dev:/dev -v $(pwd):/mnt goldarte/img-tool:v0.5 img-chroot /mnt/<IMAGE>
```

где `<IMAGE>` - имя файла образа. В открывшемся терминале с помощью стандартных программ (nano, git, cp, apt-get) вы можете донастроить образ.

* Внешние файлы вы можете перенести в образ с помощью команды:

```bash
sudo docker run --privileged -it --rm -v /dev:/dev -v $(pwd):/mnt goldarte/img-tool:v0.5 img-chroot /mnt/<IMAGE> copy /mnt/<MOVE_FILE> <MOVE_TO>
```

где `<MOVE_FILE>` - файл, который нужно перенести в образ (расположение относительно папки с образом, например `../builder/assets/clever-show.service`), а `<MOVE_TO>` - путь в образе, куда нужно переместить файл.

* Если в образе не хватает места для всех необходимых файлов, можно расширить образ с помощью команды:

```bash
sudo docker run --privileged -it --rm -v /dev:/dev -v $(pwd):/mnt goldarte/img-tool:v0.5 img-resize /mnt/<IMAGE> max <SIZE>
```

где `<SIZE>` - размер в байтах. Например 5G будет означать 5GB, а 5M - 5MB.

* После расширения образа его можно сжать до минимального размера + 10МB командой

```bash
sudo docker run --privileged -it --rm -v /dev:/dev -v $(pwd):/mnt goldarte/img-tool:v0.5 img-resize /mnt/<IMAGE> min
```

## Изменение скриптов сборки

Статья по изменению скриптов сборки образа и создания кастомной сборки написана [здесь](https://clever.copterexpress.com/ru/image_building.html)