# prosoft-c-stack
Class project for C-language lessons, Prosoft Developer School 2022

## Краткое описание

Проект предназначен для тривиальной реализации структуры данных
[стек](https://github.com/czertyaka/prosoft-c-stack/blob/master/docs/stack.md)
в учебных целях.

В проект включена статическая библиотека `cstack`, написанная на C и реализующая структуру данных.
Для библиотеки определен интерфейс (API) в виде набора прототипов функций в файле
[cstack.h](https://github.com/czertyaka/prosoft-c-stack/blob/master/cstack.h).
В файле [cstack.c](https://github.com/czertyaka/prosoft-c-stack/blob/master/cstack.c)
содержатся пустые определения этих функций.
Вам необходимо дополнить эти определения таким образом, чтобы библиотека удовлетворяла требованиям
[технического задания](https://github.com/czertyaka/prosoft-c-stack/blob/master/docs/technical-requirement.md).

В проекте используется система сборки [cmake](https://cmake.org/).
Это одна из самых популярных систем сборки для проектов на C/C++.
Поддержка `cmake` имеется во многих популярных IDE (Visual Studio, QtCreator и др.).
Сборка проекта описана в файле
[CMakeLists.txt](https://github.com/czertyaka/prosoft-c-stack/blob/master/CMakeLists.txt).
Порядок сборки проекта с примерами в разных средах разработки описан в
[этом разделе](https://github.com/czertyaka/prosoft-c-stack/blob/master/docs/build.md).
Подробнее о системах сборки
[тут](https://habr.com/ru/articles/155201/)
и [тут](https://habr.com/ru/articles/155467/)
(порядок важен).

Имеется возможность (по-умолчанию отключена) собрать исполняемый файл для ее тестирования
([test.cpp](https://github.com/czertyaka/prosoft-c-stack/blob/master/test.cpp)).
Для включения теста в сборку проекта необходимо сконфигурировать `cmake` с включенной опцией `WITH_TEST`.
Код теста написан на C++, используется библиотека [GoogleTest](https://github.com/google/googletest).
Тест будет собран с использованием
[санитайзеров](https://en.wikipedia.org/wiki/Code_sanitizer).

## Зависимости
* `gcc` или `clang` — компиляторы C/С++.
* `cmake >= 3.22` — генератор скриптов сборки make/Ninja/etc.
* `gtest` (опционально) — библиотека для написания модульных тестов.

Понадобится также система сборки (make/Ninja/etc) по выбору.

## Каналы общения
Корпоративная почта: o.pyhov@prosoftsystems.ru  
Telegram: @czert  

## Документация
Не все разделы документации обязательны для ознакомления,
если обсуждаемые в них темы вам уже знакомы.
Более того, некоторые из тем носят чисто факультативный характер.
* [Описание структуры данных стек](https://github.com/czertyaka/prosoft-c-stack/blob/master/docs/stack.md).
Если вы не знаете, как работает стек, то этот раздел к прочтению обязателен.
* [Техническое задание](https://github.com/czertyaka/prosoft-c-stack/blob/master/docs/technical-requirement.md).
Содержит непосредственно техническое задание и требования к реализации (единственная по-настоящему
обязательная часть сопровождения), подсказки к написанию кода библиотеки и пошаговая инструкция
выполнения домашнего задания.
Процесс сдачи и оценивания ДЗ основан на работе с системой контроля версий Git и платформой GitHub,
так что требования к работе с этими инструментами также являются частью технического задания.
Другими словами, строго придерживаемся пошаговой инструкции и не присылаем мне код на почту
:smiling_face_with_tear:.
* [Инструкция по сборке](https://github.com/czertyaka/prosoft-c-stack/blob/master/docs/build.md).
В этом разделе пошагово со скриншотами воспроизвожу сборку проекта без тестов и с ним для
командной строки Linux, в Visual Studio и QtCreator.
Для VSCode с расширениями инструкции нет, поскольку набор расширений у каждого индивидуальный,
и универсальную инструкцию сделать невозможно.
