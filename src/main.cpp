// ============================================================
// HoloPhysics Lab 2 — Точка входа
// Научная физическая песочница с управлением жестами
// C++20 / OpenGL 4.6 / Dear ImGui / Hand Tracking
// ============================================================

#include "HoloPhysicsApp.h"
#include <iostream>

int main(int argc, char** argv) {
    (void)argc; (void)argv;
    
    hlp::HoloPhysicsApp app;
    
    if (!app.init()) {
        std::cerr << "❌ Ошибка инициализации HoloPhysics Lab 2" << std::endl;
        return -1;
    }
    
    std::cout << R"(
╔══════════════════════════════════════════════════╗
║         HoloPhysics Lab 2 — v2.5               ║
║    Научная физическая песочница                 ║
║    C++20 / OpenGL 4.6 / ImGui / Hand Tracking   ║
╚══════════════════════════════════════════════════╝

📋 Управление:
   Пробел   — Пауза / Продолжить
   R        — Сбросить эксперимент
   F        — Тоггл гравитационного колодца
   X        — Взрыв
   1-9      — Быстрый выбор пресета
   Delete   — Удалить выбранное тело
   Escape   — Выход
   Middle+  — Вращение камеры
   Scroll   — Zoom

)" << std::flush;
    
    if (app.ar_mode_enabled()) {
        std::cout << R"(   🤚 AR РЕЖИМ АКТИВЕН:
   Щипок (Pinch)      — Захватить тело
   Кулак (Fist)        — Оттолкнуть всё
   Указание (Pointing) — Выбрать тело
   Ладонь (Open Palm)  — Силовое поле

)" << std::flush;
    }
    
    app.run();
    
    std::cout << "👋 До свидания!" << std::endl;
    return 0;
}
