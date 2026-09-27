# Источники и происхождение

- [Изложение постановки «Резервная одометрия по модели», уточнений и геометрии](task/README.md).
- Стартовый архив tram_solution_v1: источник модели B6, обученных деревьев и исходного разделения по датам. Результаты перепроверены на исходных данных; прежний holdout уже был раскрыт и называется аудитом.
- Два исходных Pathgraph (не публикуются; порядок использования — [data/README.md](../data/README.md)), предоставленные организаторами; [SHA-256 исходников](../config/input_sources.json), [замороженное преобразование TRAIN](../config/map_calibration.json). `scripts/restore_official_map.py` восстанавливает оба преобразованных CSV из исходных JSON с совпадением SHA-256.
- Геометрия трамвая по уточнениям организаторов: master=(-9.873,0,3), rover=(2.563,0,3), расстояние между тележками7.55м; base_link у передней тележки на высоте рельса.
- [ROS Humble QoS](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst).
- [ROS Humble create_timer](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/create_timer.hpp).
- [Определение nav_msgs/Odometry и системы координат twist](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg).
- [Официальный тест SequentialWriter rosbag2 Humble](https://github.com/ros2/rosbag2/blob/humble/rosbag2_py/test/test_sequential_writer.py): API создания синтетического SQLite-bag для проверки воспроизведения.

Производные терминальные участки обучены только на 30618_0652866c и 30618_073f08d1; основная геометрия маршрута взята из официального Pathgraph. Код не выбирает маршрут по идентификатору или времени записи.
