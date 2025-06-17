#include "sensors/common/utils.h"

void check_element_creation(GstElement *element, const std::string &name) {
    if (!element) {
        std::cerr << "Failed to create " << name << " element." << std::endl;
    } else {
        std::cout << name << " element created successfully." << std::endl;
    }
}

void check_linking(bool success, const std::string &linking) {
    if (!success) {
        std::cerr << "Failed to link " << linking << "." << std::endl;
    } else {
        std::cout << "Successfully linked " << linking << "." << std::endl;
    }
}
