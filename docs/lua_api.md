# HoloPhysics Lab 2 — Lua Scripting API

**Версия**: 3.6.0
**Модуль**: `src/scripting/ScriptEngine.h`
**Зависимость**: Lua 5.4 + sol2

---

## 📋 Обзор

Lua-скриптинг позволяет управлять всеми аспектами симуляции без перекомпиляции C++ кода. API предоставляет 30+ функций для создания объектов, управления физикой, обработки событий и отладки.

---

## 🚀 Быстрый старт

```lua
-- my_experiment.lua
function on_start()
    print("Эксперимент запущен!")
    set_gravity(5.0)
    
    -- Создать 10 планет
    for i = 1, 10 do
        local x = math.cos(i * 0.63) * 7
        local z = math.sin(i * 0.63) * 7
        create_sphere(x, 0, z, 0.5 + i * 0.1, 0.2 + i * 0.02)
    end
end

function on_update(dt)
    local t = get_time()
    set_gravity(9.81 * (1.0 + 0.5 * math.sin(t * 2.0)))
end

function on_collision(a, b, impulse)
    if impulse > 10 then
        apply_impulse(a, 0, 10, 0)
        print("Сильное столкновение: " .. a .. " × " .. b)
    end
end
```

---

## 🔧 Функции API

### Создание и управление телами

| Функция | Сигнатура | Описание |
|---------|-----------|----------|
| `create_sphere` | `(x, y, z, mass, radius) → entity_id` | Создать сферу |
| `create_box` | `(x, y, z, mass, size) → entity_id` | Создать куб |
| `create_spring` | `(a, b, stiffness, damping) → bool` | Создать пружину |
| `remove_body` | `(entity_id)` | Удалить тело |
| `clear_all` | `()` | Удалить все тела |

### Чтение/запись свойств

| Функция | Сигнатура | Описание |
|---------|-----------|----------|
| `get_position` | `(entity_id) → x, y, z` | Позиция тела |
| `set_position` | `(entity_id, x, y, z)` | Установить позицию |
| `get_velocity` | `(entity_id) → vx, vy, vz` | Скорость тела |
| `set_velocity` | `(entity_id, vx, vy, vz)` | Установить скорость |
| `get_mass` | `(entity_id) → mass` | Масса тела |
| `set_mass` | `(entity_id, mass)` | Установить массу |

### Силы и импульсы

| Функция | Сигнатура | Описание |
|---------|-----------|----------|
| `apply_force` | `(entity_id, fx, fy, fz)` | Приложить силу |
| `apply_impulse` | `(entity_id, jx, jy, jz)` | Приложить импульс |
| `apply_force_at` | `(entity_id, fx, fy, fz, px, py, pz)` | Сила в точке |
| `explode` | `(cx, cy, cz, radius, force)` | Взрыв |

### Параметры симуляции

| Функция | Сигнатура | Описание |
|---------|-----------|----------|
| `get_gravity` | `() → g` | Текущая гравитация |
| `set_gravity` | `(g)` | Установить гравитацию |
| `set_damping` | `(d)` | Установить затухание |
| `get_time` | `() → t` | Время симуляции |
| `get_body_count` | `() → count` | Количество тел |
| `get_fps` | `() → fps` | FPS |

### Пресеты и сцены

| Функция | Сигнатура | Описание |
|---------|-----------|----------|
| `load_preset` | `(name)` | Загрузить пресет |
| `save_scene` | `(filename)` | Сохранить сцену |
| `load_scene` | `(filename)` | Загрузить сцену |
| `list_presets` | `() → table` | Список пресетов |

### Запросы

| Функция | Сигнатура | Описание |
|---------|-----------|----------|
| `find_nearest` | `(x, y, z, radius) → entity_id` | Найти ближайшее тело |
| `raycast` | `(ox, oy, oz, dx, dy, dz) → entity_id, hx, hy, hz` | Пустить луч |
| `get_bodies_in_radius` | `(x, y, z, r) → table` | Тела в радиусе |

### Рендеринг

| Функция | Сигнатура | Описание |
|---------|-----------|----------|
| `set_background_color` | `(r, g, b)` | Цвет фона |
| `set_fog` | `(density)` | Плотность тумана |
| `set_camera_target` | `(x, y, z)` | Цель камеры |
| `set_camera_distance` | `(d)` | Дистанция камеры |

### Ввод

| Функция | Сигнатура | Описание |
|---------|-----------|----------|
| `is_key_pressed` | `(key) → bool` | Клавиша нажата? |
| `get_mouse_position` | `() → x, y` | Позиция мыши |
| `get_hand_gesture` | `() → gesture` | Жест руки |
| `get_hand_position` | `(hand_index) → x, y, z` | Позиция руки |

### Отладка

| Функция | Сигнатура | Описание |
|---------|-----------|----------|
| `print` | `(...)` | Вывод в консоль |
| `log` | `(message)` | Логирование |
| `draw_line` | `(x1,y1,z1, x2,y2,z2, r,g,b)` | Линия отладки |
| `draw_text` | `(x,y,z, text, r,g,b)` | Текст в 3D |
| `benchmark_start` | `()` | Начать замер |
| `benchmark_end` | `() → ms` | Закончить замер |

---

## 🔄 События (колбэки)

| Событие | Сигнатура | Частота |
|---------|-----------|---------|
| `on_start()` | `()` | 1 раз при загрузке |
| `on_update(dt)` | `(dt)` | Каждый кадр (60 Гц) |
| `on_fixed_update(dt)` | `(dt)` | Каждый шаг физики (240 Гц) |
| `on_collision(a, b, impulse)` | `(a, b, impulse)` | При столкновении |
| `on_grab(body_id)` | `(body_id)` | При захвате |
| `on_release(body_id, vx, vy, vz)` | `(body_id, vx, vy, vz)` | При отпускании |
| `on_key_press(key)` | `(key)` | При нажатии клавиши |
| `on_gesture(gesture)` | `(gesture)` | При жесте |
| `on_scene_load(name)` | `(name)` | При загрузке сцены |

---

## 📊 Примеры скриптов

### 1. Осциллирующая гравитация
```lua
function on_start()
    initial_g = get_gravity()
end
function on_update(dt)
    local t = get_time()
    set_gravity(initial_g * (1 + 0.5 * math.sin(t * 2)))
end
```

### 2. Случайные тела каждые 5 секунд
```lua
local timer = 0
function on_update(dt)
    timer = timer + dt
    if timer > 5 then
        timer = 0
        for i = 1, 10 do
            create_sphere(
                math.random(-8, 8),
                math.random(2, 10),
                math.random(-8, 8),
                1, 0.3
            )
        end
        print("Создано 10 тел")
    end
end
```

### 3. Антигравитационная зона
```lua
function on_update(dt)
    local bodies = get_bodies_in_radius(0, 5, 0, 10)
    for _, id in ipairs(bodies) do
        local x, y, z = get_position(id)
        if y < 5 then
            apply_force(id, 0, 10, 0)  -- Отталкивание вверх
        end
    end
end
```

### 4. Batch-симуляция
```lua
-- parameter_sweep.lua
function on_start()
    for g = 1, 10 do
        set_gravity(g * 2)
        run_simulation(1000)  -- Встроенная функция
        local e = get_total_energy()
        log("Gravity=" .. (g*2) .. " Energy=" .. e)
    end
end
```
