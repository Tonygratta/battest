Проект автоматизированного измерителя параметров Ni-Cd аккумуляторов.
Создавался для задачи отбора из большого количества неликвида, по случаю оказавшегося в наличии,
наилучших экземпляров аккумуляторов для сборки из них батареи для шуруповёрта.
А также для получения известного удовольствия от возни с Arduino, паяльником, и постижения азов
работы с GitHub и программирования на С.
Аппаратная часть проекта выполнена в форм-факторе платы расширения, монтируемой на Arduino Uno,
на макетной плате. Обеспечивает стабилизированный ток зарядки 180 мА и ток разряда около 600 мА.
Требует питания 12 В, потребляет ток около 190 мА при зарядке, 10 мА при разряде.
Цикл заряд - разряд занимает 19 ч для аккумуляторов ёмкостью 1500 мА*ч (SC).
Проект отлажен на Arduino Nano ATMEGA328P (CH340G) через адаптер Arduino Nano/Uno.
Схема будет приведена в файле scheme.jpg, когда я её дорисую.
