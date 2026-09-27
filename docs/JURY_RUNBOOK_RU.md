# Запуск и проверка в ROS 2 Humble

Инструкция относится к **0.6**. Окружение: Ubuntu 22.04, ROS 2 Humble, C++17, CMake и colcon. Карты и веса включены в репозиторий. Исходный bag и числовой fixture организаторов не публикуются; проверка на своём bag описана ниже.

## 1. Зависимости и сборка

Нужны `ament_cmake`, `rclcpp`, `std_msgs`, `sensor_msgs`, `nav_msgs`, `rosidl_default_generators`, `rosidl_default_runtime`, `launch_ros` и `ament_index_python`. Для официального чекера нужен `message_filters` (`ros-humble-message-filters`), для вспомогательных проверок — Python ≥ 3.10 и NumPy. Matplotlib нужен только для графиков.

```bash
git clone https://github.com/Nibani/tram-odometry-final.git
cd tram-odometry-final
git checkout main
source /opt/ros/humble/setup.bash
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --executor sequential
colcon test-result --verbose
```

Сборка не устанавливает зависимости и не скачивает веса. Для автономного компьютера подготовьте ROS-окружение заранее. В workspace должна быть одна копия `tram_vehicle_msgs` с определением `DriverControllerCommand`: сокращённый пакет из примера чекера этого сообщения не содержит.

## 2. Запуск узла, чекера и rosbag

В каждом терминале загрузите одно и то же окружение:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Сначала запустите узел:

```bash
ros2 launch reserve_odometry odometry.launch.py use_sim_time:=true map_mode:=calibrated output_frame:=map
```

В другом терминале запустите неизменённый официальный чекер:

```bash
mkdir -p results/jury_manual
python3 tests/vendor/official_metrics.py --ros-args -p use_sim_time:=true 2>&1 | tee results/jury_manual/official_metrics.log
```

При использовании пакета организаторов эквивалентная команда — `ros2 run hackathon_solution_checker metrics --ros-args -p use_sim_time:=true`. `relay_result.py` для этой проверки запускать нельзя: он копирует эталон в результат.

До воспроизведения можно включить запись выходов в отдельном терминале:

```bash
ros2 bag record -o results/jury_manual/output_bag /result/velocity /result/position /result/slip_status /result/state_status /result/diagnostics
```

Затем запустите запись организаторов:

```bash
ros2 bag play /absolute/path/to/bag --clock
```

Путь указывает на каталог одной записи с `metadata.yaml` и `.db3`. Каталог для записи результатов должен быть новым. Для следующего независимого заезда перезапустите узел и чекер. Обратный скачок ROS-времени сбрасывает состояние узла; перемотка вперёд не заменяет новый запуск. Для живых сенсоров используйте `use_sim_time:=false`.

## 3. Входы и выходы

| Входной топик | Тип | Содержание |
|---|---|---|
| `/vehicle/front_bogie_velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | Скорость передней тележки |
| `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | Скорость задней тележки |
| `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/msg/DriverControllerCommand` | Положение контроллера −15…15 |
| `/sensing/gnss/master/fix` | `sensor_msgs/msg/NavSatFix` | Положение антенны master |
| `/sensing/gnss/rover/fix` | `sensor_msgs/msg/NavSatFix` | Положение антенны rover |

Для предоставленного колёсного потока выбран делитель 3,6. После нормализации скорость измеряется в м/с. GNSS задаёт начальную абсолютную привязку и разрешённые редкие коррекции. Подписок на GNSS-скорость и `/localization/kinematic_state` в узле нет; эталон читает только чекер.

| Выходной топик | Тип | Содержание |
|---|---|---|
| `/result/velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | Оценка скорости, м/с |
| `/result/position` | `nav_msgs/msg/Odometry` | XYZ `base_link` в системе карты, ориентация и twist кузова |
| `/result/slip_status` | `std_msgs/msg/UInt8` | Младшие восемь бит состояния |
| `/result/state_status` | `std_msgs/msg/UInt16` | Полная маска состояния |
| `/result/diagnostics` | `std_msgs/msg/Float64MultiArray` | Время вычисления, маршрут, пробег и диагностика привязки |

В конкурсном запуске используются `frame_id=map`, `child_frame_id=base_link`, нулевые `output_point_x/z`. Точка `base_link` находится на оси передней тележки на высоте контакта с рельсом. Чекер сравнивает XYZ напрямую, без преобразований TF.

До успешной абсолютной привязки позиция не публикуется. Скорость и диагностика продолжают поступать. Покрытие и время первой позиции нужно оценивать вместе с ошибкой; отсутствие координат не означает нулевую ошибку.

## 4. Диагностика

```bash
ros2 topic hz /result/velocity
ros2 topic echo /result/position --once
ros2 topic echo /result/state_status
ros2 topic echo /result/diagnostics
```

Полная расшифровка маски и полей массива находится в [PARAMETERS_RU.md](PARAMETERS_RU.md). Диагностика показывает недоверие каждому колесу, состояние начальной привязки, устаревший контроллер и разрывы обновлений. Эти признаки описывают решения наблюдателя, а не измеренную вероятность проскальзывания.

Публикации обычно следуют сообщениям контроллера; при его потере включается резервный таймер 20 Гц. Реальную частоту следует измерять на целевой машине. Поле `elapsed_us` в диагностике измеряет локальную работу узла и не включает полную задержку DDS.

## 5. Полный DDS-прогон и результаты

После сборки подготовьте числовой fixture из собственного файла `.db3` (нужны NumPy и стандартная библиотека Python):

```bash
python3 scripts/check_official.py decode --bag /absolute/path/to/bag/file.db3 --out results/decoded
python3 scripts/make_checker_fixture.py --decoded results/decoded --out tests/fixtures/checker_sample.npz
python3 tests/ros_checker_replay.py --fixture tests/fixtures/checker_sample.npz --out results/jury_replay --rate 1 --wall-timeout 1450
```

Сценарий воспроизводит подготовленный числовой пример через ROS и запускает официальный чекер. Файл `tests/fixtures/checker_sample.npz` не включён в публичный выпуск: его можно получить из своего исходного bag приведёнными командами. Без него полный прогон в публичном CI пропускается; сборка, native и синтетические DDS-проверки выполняются. Для полного прогона при темпе 1× требуется около 22 минут. Без `--rate 1` скрипт по умолчанию использует ускоренный режим 4×; его результаты быстродействия нужно подписывать отдельно.

| Файл | Что проверить |
|---|---|
| `results/jury_replay/checker_dds.json` | Ошибки, покрытие, частоту, задержку command→result и ресурсы узла |
| NPZ-файлы в том же каталоге | Сопоставленные пары и сырые выходы |
| `results/jury_replay/estimator_node.log` | Лог алгоритма |

RMSE и максимум ошибки скорости измеряются в м/с, позиции — в метрах по XYZ. Для производительности нужны частота, p99 и максимум задержки, RSS и загрузка CPU с указанными условиями измерения. Требования задания: не менее 10 Гц, предпочтительно 20–50 Гц, задержка до 100 мс и пиковая до 250 мс, не более двух ядер и 0,5 ГБ памяти.

[Измеренные результаты, таблицы и графики](ACCURACY_RU.md) · [CI для проверенной версии](https://github.com/Nibani/tram-odometry-final/actions?query=branch%3Amain)

## 6. Дополнительные проверки и режимы

```bash
python3 tests/ros_checker_compat.py --out results/jury_checker_compat
python3 tests/ros_late_pair_recovery.py --out results/jury_late_pair
python3 tests/ros_startup_clock_backlog.py --out results/jury_startup_clock
python3 scripts/check_official.py selftest
```

Эти сценарии проверяют совместимость с чекером, восстановление начальной привязки по поздней паре GNSS и порядок обработки сообщений при запуске ROS-времени. Они дополняют полный прогон.

Для проверки влияния GNSS-поправок передайте `sparse_gnss_correction:=false`. `map_mode:=single` оставляет одну подготовленную карту, `official` использует исходные открытые участки, `none` включает относительную одометрию. Режимы и их параметры описаны в [PARAMETERS_RU.md](PARAMETERS_RU.md). Для официальной оценки абсолютного XYZ используйте штатный `map_mode:=calibrated output_frame:=map`.

Порядок использования своих исходных записей и контрольные суммы описаны в [инструкции получения данных](../data/README.md). `scripts/reproduce.py` воспроизводит историческую оценку на прежнем наборе; её условия следует читать отдельно от текущего официального DDS-прогона. Офлайн-проверка `scripts/check_official.py` удобна для повторного расчёта ошибок, но не измеряет реальную задержку доставки ROS-сообщений.
