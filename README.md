# Исследование использования монокулярной оценки глубины для улучшения визуально-инерциальной одометрии.
# Участники: 

Горохов Сергей R4135с 

Овсянников Семён R4136с 

Новрузов Рафиг R4134с

Ларионов Александр R4134с

Караусов Александр R4135с

# Описание проводимого исследования:

## Постановка задачи:

- Предлагается интегрировать данные о глубине от нейросети в VIO-систему. 
    
- Исследовательская задача — модифицировать ядро фильтра Калмана (EKF) для динамического взвешивания остатков (residuals). 
    
- Фильтр должен адаптивно изменять доверие к VIO-признакам или данным о глубине в зависимости от их надежности. 
    
- Примечание: задача требует модификации C++ кода.
    

## Описание решенных задач:

- Реализовано рабочее окружение: Ubuntu 22.04 + ROS2 Humble + OpenVINS.
    
- Исследованы теоретические основы VIO и расширенного фильтра Калмана с учётом pre-integration IMU и скользящего окна поз камеры (MSCKF).
    
- Интегрированы три источника информации о глубине: RGB-D камера RealSense, нейросетевая глубина (Depth Anything / Depth Anything 3) и триангуляция в OpenVINS.
    
- Реализована гибридная схема адаптивного взвешивания измерений глубины с учётом доверия сенсоров, расстояния и текстурности сцены.
    

## Распределение задач:

- Горохов Сергей - считывание данных с IMU и их связь и синхронизация с камерой.
    
- Овсянников Семён - реализация работы OpenVINS в docker с камерой RealSense и датчиком IMU.
    
- Новрузов Рафиг - работа с нейросетевой моделью Depth Anything, оформление ее как ROS-топик и визуализация  работы алгоритма на датасете.
    
- Ларионов Александр Юрьевич - работа с оборудованием/камерой  и тестирование OpenVINS.
    
- Караусов Александр - фильтр Калмана в OpenVINS + презентация.

# Depth-Anything-3 

**Depth-Anything-3** — это модуль для построения карт глубины с помощью нейросети на основе изображения с камеры. Модуль использует предварительно обученные веса модели (`model.safetensors`) и позволяет получать карту глубины в реальном времени.

<p align="center">
  <img src="assets/images/demo320-2.gif" alt="Depth Anything 3 - Left" width="70%">
</p>

## Основные возможности 

- Получение изображения с камеры 
- Генерация карты глубины с использованием модели `Depth-Anything-3` 
- Простая интеграция в Python-проекты 

## Установка и запуск на Windows

1. **Скачать zip-архив с ветки `master1`** 
   - Перейдите на GitHub репозиторий и скачайте ZIP-архив с ветки `master1`.

2. **Распаковать архив** 
   - Распакуйте содержимое архива в удобную папку.

3. **Скачать веса модели** 
   - Скачайте `model.safetensors` по ссылке:  
     [https://huggingface.co/depth-anything/DA3METRIC-LARGE/tree/main](https://huggingface.co/depth-anything/DA3METRIC-LARGE/tree/main)  
   - Поместите файл в папку:  
     ```
     Depth-Anything-3-main\src\depth_anything_3\weights\DA3METRIC-LARGE
     ```

4. **Скачать и установить Python 3.9.9** 
   - Ссылка для скачивания: [Python 3.9.9](https://www.python.org/downloads/release/python-399/)

5. **Создать виртуальное окружение**  
   - Откройте папку проекта в VS Code  
   - Создайте виртуальное окружение, указав путь к Python 3.9.9:
     ```bash
     "C:\...\Python39\python.exe" -m venv .venv
     ```

6. **Активировать виртуальное окружение**  
   ```bash
   .venv\Scripts\activate
   ```

7. **Установить необходимые зависимости**   
   ```bash
   pip install -r requirements.txt
   ```

8. **Установите текущую папку как Python-пакет**   
   ```bash
   pip install .
   ```

9. **Запуск файла**  
   ```bash
   python infer_camera.py
   ```

# Настройка рабочего окружения: Ubuntu 22.04 + ROS2 Humble + OpenVINS

## [Инструкция по установке ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
   Устанавливаем ros-humble-desktop и rov-dev-tools.

## Установка OpenVINS

1. **Подготовка рабочего пространства** 
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **Клонирование OpenVINS**
   ```bash
   git clone https://github.com/rpng/open_vins.git
   ```

3. **Установка зависимостей**
   ```bash
   sudo apt update
   sudo apt install -y libgoogle-glog-dev libgflags-dev libsuitesparse-dev libceres-dev libopencv-dev
   ```

4. **Установка зависимостей ROS через rosdep**
   ```bash
   cd ~/ros2_ws
   sudo apt install python3-rosdep
   sudo rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. **Закрываем терминал**

6. **Увеличиваем SWAP (файл подскачки) до максимума (16 ГБ)**
   ```bash
   sudo swapoff -a  # Выключить текущий
   sudo fallocate -l 16G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   ```

7. **Запускаем сборку в один поток**
   ```bash
   source /opt/ros/humble/setup.bash
   cd ~/ros2_ws
   colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_PARALLEL_LEVEL=1
   ```

## Проверка корректности установки OpenVINS

1. **Запускаем новый терминал**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
   ```

2. **Запускаем еще один терминал для запуска RViz**
   ```bash
   source ~/ros2_ws/install/setup.bash
   rviz2
   ```
   После запуска rviz в левом нижнем углу жмем на кнопку Add, переходим во вкладку by topic и проверяем, что отображаются топики ov_msckf (OpenVINS), что говорит о корректной установке.

## Запуск тестового датасета OpenVINS

1. **Скачиваем датасет MH_01_easy (rosbag2) по [ссылке](https://docs.openvins.com/gs-datasets.html#gs-data-euroc)**

2. **Распаковываем zip-архив с датасетом, получаем папку в Downloads**

3. **Запускаем OpenVINS**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
   ```

4. **Запускаем RViz в новом терминале**
   ```bash
   source ~/ros2_ws/install/setup.bash
   rviz2
   ```

5. **В окне RViz слева в меню Fixed Frame пишем global**

6. **Также в левом нижнем углу жмем на кнопку Add и добавляем 2 топика: /ov_msckf/pathimu и /ov_msckf/poseimu**

7. **Запускаем датасет в новом терминале**
   ```bash
   cd ~/Downloads
   source ~/ros2_ws/install/setup.bash
   ros2 bag play MH_01_easy 
   ```

## Сборка нейросети Depth Anything 3 на Ubuntu 22.04

1. **Скачать zip-архив с ветки `master1`** 
   
   Перейдите на GitHub репозиторий и скачайте ZIP-архив с ветки `master1`.

2. **Распаковать архив** 
  
   Распакуйте содержимое архива в удобную папку.

3. **Скачать веса модели** 

   Скачайте `model.safetensors` по ссылке:  
     [https://huggingface.co/depth-anything/DA3METRIC-LARGE/tree/main](https://huggingface.co/depth-anything/DA3METRIC-LARGE/tree/main)  
   Поместите файл в папку:  
     ```
     Depth-Anything-3-main\src\depth_anything_3\weights\DA3METRIC-LARGE
     ```

4. **Создать виртуальное окружение**  
   Откройте папку проекта в VS Code  
   Создайте виртуальное окружение, использую системный Python (для Ubuntu 22.04 это Python 3.10):
     ```bash
     /usr/bin/python3 -m venv .venv
     ```

5. **Активировать виртуальное окружение** 
   ```bash
   source .venv/bin/activate
   ```

6. **Установить необходимые зависимости**  
   ```bash
   pip install -r requirements.txt
   ```

7. **Установите текущую папку как Python-пакет**  
   ```bash
   pip install .
   ```

## Порядок запуска рабочего окружения

1. **Запуск OpenVINS**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
   ```

2. **Запуск RViz**
   ```bash
   source ~/ros2_ws/install/setup.bash
   rviz2
   ```

3. **В окне RViz слева в меню Fixed Frame пишем camera**

4. **Запуск ROS и python-скрипта**  
   ```bash
   source ~/ros2_ws/install/setup.bash
   python3 depth_publisher.py
   ```

5. **В окне RViz в левом нижнем углу жмем на кнопку Add и добавляем 2 топика: /cam0/image_raw и /depth/image_raw**

6. **Запуск датасета в новом терминале**
   ```bash
   cd ~/Downloads
   source ~/ros2_ws/install/setup.bash
   ros2 bag play MH_01_easy 
   ```





