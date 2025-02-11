
```bash
ros2 launch fairino_mtc_demo mtc_builder.launch.py
ros2 launch fairino_mtc_demo moveit_sim.launch.py

ros2 run fairino_mtc_demo task_generator joints_move 0 -1.57 1.57 0 0 0
rosrun moveit_python task_generator.py help
rosrun moveit_python task_generator.py robot get_robot_param
rosrun moveit_python task_generator.py fr10 joints_move
rosrun moveit_python task_generator.py fr10 joints_move 0 0 0 0 0 0
rosrun moveit_python task_generator.py fr10 absolute_move absolute_move
rosrun moveit_python task_generator.py fr10 absolute_move hello_box
rosrun moveit_python task_generator.py fr10 spawn_object hello_box 0 0.5 0.2
rosrun moveit_python task_generator.py fr10 attach_object hello_box absolute_move
rosrun moveit_python task_generator.py fr10 detach_object hello_box absolute_move
rosrun moveit_python task_generator.py fr10 remove_object hello_box
rosrun moveit_python task_generator.py fr10 clear_scene
rosrun moveit_python task_generator.py fr10 gripper_open
rosrun moveit_python task_generator.py fr10 gripper_close
rosrun moveit_python task_generator.py fr10 choose_pipeline OMPL RRTConnect
rosrun moveit_python task_generator.py fr10 choose_pipeline PILZ LIN
rosrun moveit_python task_generator.py fr10 choose_follow_mode
rosrun moveit_python task_generator.py fr10 check_json_files
rosrun moveit_python task_generator.py fr10 delete_json_sim_content test.json
rosrun moveit_python task_generator.py fr10 delete_json_temp
```


Добавлю функциональность, которая может быть полезной для развития вашего проекта. Вот расширенный список функций:

### Управление сценой
- **SAVE_SCENE**: сохранение текущей конфигурации объектов сцены в файл.
- **LOAD_SCENE**: загрузка сохранённой конфигурации объектов сцены из файла.
- **RESET_SCENE**: сброс сцены к первоначальному состоянию.

### Управление объектами
- **EDIT_OBJECT_PROPERTIES**: редактирование свойств объекта (размер, материал, цвет и т.д.).
- **DUPLICATE_OBJECT**: создание копии выбранного объекта.
- **MOVE_OBJECT**: перемещение объекта в заданное место.

### Управление движениями
- **RELATIVE_MOVE**: движение относительно текущей позиции.
- **MOVE_TO_HOME**: возврат в исходное положение.
- **ROTATE_OBJECT**: вращение объекта вокруг оси.

### Роботизированное управление
- **SET_SPEED**: установка скорости движения робота.
- **SET_ACCELERATION**: настройка ускорения робота.
- **PAUSE_MOTION**: временная приостановка движения.
- **RESUME_MOTION**: возобновление движения после паузы.
- **STOP_MOTION**: остановка текущего движения робота.

### Манипуляции
- **GRIPPER_FORCE_ADJUST**: регулировка силы захвата.
- **GRIPPER_STATE**: получение текущего состояния захвата.

### Работа с сенсорами
- **START_SENSOR_STREAM**: запуск потоковой передачи данных с сенсоров.
- **STOP_SENSOR_STREAM**: остановка потоковой передачи данных.
- **GET_SENSOR_DATA**: получение данных с выбранного сенсора.

### Калибровка и настройка
- **CALIBRATE_ROBOT**: полная калибровка робота.
- **SET_WORKSPACE_BOUNDS**: установка границ рабочей области.

### Работа с программами
- **IMPORT_GCODE**: импорт G-кода для выполнения сложных задач.
- **EXPORT_TRAJECTORY**: экспорт текущей траектории в файл.
- **RUN_SCRIPT**: запуск пользовательских скриптов.

### Визуализация и отладка
- **ENABLE_DEBUG_MODE**: включение режима отладки.
- **DISABLE_DEBUG_MODE**: отключение режима отладки.
- **SHOW_SCENE_STATUS**: отображение текущего статуса сцены (например, список объектов, их положение и свойства).

### Работа с пользователями
- **SET_USER_ACCESS**: установка прав доступа для разных пользователей.
- **LOG_USER_ACTIONS**: ведение журнала действий пользователей.

Добавление этих функций обеспечит более полное покрытие потребностей вашего проекта и улучшит его функциональность. Если нужно реализовать подробности по любой из них, уточните, и я помогу с этим!s