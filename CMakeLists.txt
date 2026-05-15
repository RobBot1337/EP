cmake_minimum_required(VERSION 3.16)
project(EP VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Поиск Qt
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

# Проверка наличия файлов
set(PROJECT_SOURCES
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
)

# Проверяем существование littlealgcpy файлов
set(LITTLEALG_SOURCES
    littlealgcpy.cpp
    littlealgcpy.h
)

# Если файлы существуют, добавляем их в проект
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/littlealgcpy.cpp" AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/littlealgcpy.h")
    list(APPEND PROJECT_SOURCES ${LITTLEALG_SOURCES})
    message(STATUS "littlealgcpy files found, adding to project")
else()
    message(WARNING "littlealgcpy.cpp or littlealgcpy.h not found!")
endif()

# Создание исполняемого файла
if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(EP
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        points.h
        points.cpp
        vec3D.h
        air_space_data.h
        airspace_loader.h
        airspace_loader.cpp
    )
else()
    if(ANDROID)
        add_library(EP SHARED ${PROJECT_SOURCES})
    else()
        add_executable(EP ${PROJECT_SOURCES})
    endif()
endif()

# Подключение библиотек
target_link_libraries(EP PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

# Настройки для iOS/macOS
if(${QT_VERSION} VERSION_LESS 6.1.0)
    set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.EP)
endif()

set_target_properties(EP PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

# Добавляем текущую директорию для поиска заголовочных файлов
target_include_directories(EP PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Установка
include(GNUInstallDirs)
install(TARGETS EP
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(EP)
endif()
