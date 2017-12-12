#include <nanogui\nanogui.h>
#include <iostream>

using namespace std;
using namespace nanogui;

enum test_enum {
	Item1 = 0,
	Item2,
	Item3
};

bool bvar = true;
int ivar = 12345678;
double dvar = 3.1415926;
float fvar = (float)dvar;
std::string strval = "A string";
test_enum enumval = Item2;
Color colval(0.5f, 0.5f, 0.7f, 1.f);


int main(int argc, char** argv) {
	nanogui::init();
	Screen* screen = new Screen(Vector2i(500, 700), "NanoGUI Example 02");

	FormHelper *gui = new FormHelper(screen);
	nanogui::ref<Window> window = gui->addWindow(Eigen::Vector2i(10, 10), "Form helper example");

	gui->addGroup("Basic types");
	gui->addVariable("bool", bvar);
	gui->addVariable("string", strval);

	gui->addGroup("Validating fields");
	gui->addVariable("int", ivar)->setSpinnable(true);
	gui->addVariable("float", fvar);
	gui->addVariable("double", dvar)->setSpinnable(true);

	gui->addGroup("Complex types");
	gui->addVariable("Enumeration", enumval, true)
		->setItems({ "Item 1", "Item 2", "Item 3" });
	gui->addVariable("Color", colval)
		->setFinalCallback([](const Color &c) {
		std::cout << "ColorPicker Final Callback: ["
			<< c.r() << ", "
			<< c.g() << ", "
			<< c.b() << ", "
			<< c.w() << "]" << std::endl;
	});

	gui->addGroup("Other widgets");
	gui->addButton("A button", []() { std::cout << "Button pressed." << std::endl; });

	screen->setVisible(true);
	screen->performLayout();
	window->center();

	nanogui::mainloop();
	
}