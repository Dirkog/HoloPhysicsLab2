#include "AppUI.h"

namespace hlp {

AppUI::AppUI() {}
AppUI::~AppUI() { shutdown(); }

bool AppUI::init(GLFWwindow* window) {
    window_ = window;
    return true;
}

void AppUI::shutdown() {}

void AppUI::new_frame() {}

void AppUI::render() {}

} // namespace hlp
