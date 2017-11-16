#include <iostream>
#include <string>
using namespace std;

template<typename T = std::string>
T f(T a = "") {
	return a;
}

//void main() {
void main_type_conversion(){
	cout << "f() : " << f() << endl;
	cout << "f(12) : " << f(12) << endl;
	cout << "f(1.2) : " << f(1.2) << endl;

	system("pause");
}
