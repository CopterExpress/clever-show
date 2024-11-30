# Подготовка среды и установка сервера в Linux

Следующая инструкция написана для сборки проекта и для запуска его с помощью Python 3.6.

К сожалению, инструкция по установке зависимостей из файла [README.md](README.md) с помощью лишь только `pip install` не работает в современных Linux с версиями python значительно выше требуемой 3.6 (например, в Ubuntu 23 стандартным пакетом, на начало 2024 года, является  Python 3.11). Проблема возникает с устаревшими зависимостями, например пакет Quamash [не поддерживается](https://pypi.org/project/Quamash/) версиями python выше 3.7.

Для запуска и сборки сервера нам потребуется [локальная версия](https://www.cherryservers.com/blog/install-python-on-ubuntu#option-2-install-python3-from-source) python 3.6, локальная версия virtual environment и подготовка их к работе по следующей инструкции. Системные файлы Linux трогать не будем, во избежание проблем с совместимостью с другими программами и актуальными пакетами Ubuntu. Соберём комплект в отдельной папке. 

## Сборка Python 3.6

Собираем Python 3.6 в Linux из исходных кодов.

```bash
# переходим в папку, в которой соберём среду для запуска сервера дронов
cd /path/to/your/directory

# создадим в ней директорию для python3.6
mkdir python3.6
cd python3.6

# установим зависимости
sudo apt install build-essential software-properties-common libssl-dev \
  libffi-dev python3-dev libgdbm-dev libc6-dev libbz2-dev libsqlite3-dev \
  tk-dev libffi-dev zlib1g-dev

# скачаем python и соберём его в локальной папке (НЕ СИСТЕМНОЙ!)
# также можно перейти по ссылке https://www.python.org/ftp/python/ и проверить
# более новые версии Python 3.6 (но в Ubuntu 23 сервер запустился в 3.6.15)
wget https://www.python.org/ftp/python/3.6.15/Python-3.6.15.tgz
tar -xvf Python-3.6.15.tgz
rm Python-3.6.15.tgz
cd Python-3.6.15

# сборка python
./configure --enable-optimizations --prefix=$(realpath ..)/install
make -j $(nproc)
# проверка, что python собрался
./python --version
# установка
make altinstall
# проверка, что python установился в локальную директорию
../install/bin/python3.6 --version
```

## Подготовка Python 3.6 Virtual Environment

Готовим среду для Python, где он будет локально запускаться и где будут расположены все его зависимости, а также пакеты для сервера дронов.

```bash
# возвращаемся в директорию в которой подготовим venv
# по инструкции выше это была директория /path/to/your/directory
cd ..

# готовим среду для Python 3.6 в директории .venv
./install/bin/python3.6 -m venv .venv
# проверяем, что и Python и pip запускаются из директории среды
.venv/bin/python3.6 --version
.venv/bin/pip3.6 --version
```

## Устанавливаем зависимости для сервера дронов и запускаем его

По инструкции выше и python 3.6 и среда для его работы уже должны быть подготовлены, а команды проверки версий должны показывать версию Python 3.6. Продолжаем настройку и запуск сервера дронов.

```bash
# возвращаемся в директорию в которой подготовим .venv
# по инструкции выше это была директория /path/to/your/directory

# ЛИБО скачиваем и распаковываем сервер дронов из архива
# wget https://github.com/CopterExpress/clever-show/archive/refs/tags/v0.4-alpha.6.tar.gz
# tar -xzvf v0.4-alpha.6.tar.gz

# ЛИБО скачиваем сервер дронов с помощью git
git clone https://github.com/CopterExpress/clever-show.git clever-show-0.4-alpha.6

# переходим в директорию, где расположен сервер дронов
cd clever-show-0.4-alpha.6/server

# устанавливаем все необходимые зависимости (команда выполняется в директории server!)
../../.venv/bin/pip3.6 install -r requirements.txt

# запускаем
../../.venv/bin/python3.6 server.py
```

## Сформированные файлы

После выполнения описанных выше действий, у вас будет сформирован комплект программного обеспечения в директории `/path/to/your/directory`:

 * `install/bin/`, `install/lib/` и т.п. - директория с Python 3.6
 * `.venv/` - директория с виртуальной средой для Python 3.6 и с установленными зависимостями сервера
 * `clever-show-0.4-alpha.6/` - директория с сервером дронов
 * `Python-3.6.15.tgz` и `Python-3.6.15/` - архив и директория в которой выполнялась сборка python из исходных кодов (можно удалить)
 * `v0.4-alpha.6.tar.gz` - архив исходных кодов сервера дронов (можно удалить)

Для запуска Python 3.6 следует пользоваться командой из инструкции выше, либо пользоваться такими путями:
 * `/path/to/your/directory/install/bin/python3.6` - путь к программе Python 3.6
 * `/path/to/your/directory/install/bin/pip3.6` - путь к программе pip 3.6
