# dumb_math
Шелонин Арсений, Б01-411, 2 курс ФРКТ


## Зависимости

| Зависимость                   | Минимальная версия    | Назначение                                    |
|-------------------------------|-----------------------|-----------------------------------------------|
| **GCC / Clang**               | GCC 10 / Clang 11     | Компиляция C++20 кода                         |
| **CMake**                     | 3.12                  | Сборка проекта и зависимостей                 |
| **libfmt-dev**                | 8.1.1                 | Парсинг форматных строк                       |
| **Boost**                     | 1.84                  | Рассчёт распределений                         |
| **graphviz**                  | 2.43.0                | Графический дамп                              |
| **NumPy, SciPy, Matplotlib**  |                       | Построение графиков                           |


### Установка зависимомтей на Ubuntu
* libfmt (устанавливается при сборке через FetchContent)
```zsh
sudo apt install libfmt-dev
```

* graphviz
```zsh
sudo apt install graphviz
```

* Boost
```zsh
wget https://archives.boost.io/release/1.84.0/source/boost_1_84_0.tar.gz
tar -xzf boost_1_84_0.tar.gz -C ~/
```


## Установка

```zsh
git clone https://github.com/71frukt/drugoe_nazvanie-numerical-methods.git
```