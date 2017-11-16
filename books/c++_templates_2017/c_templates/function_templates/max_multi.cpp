#include <iostream>
#include <string>
using namespace std;

template<typename T1, typename T2, typename RT>
RT max(T1 a, T2 b) {
	return b < a ? a : b;
}

//void main() {
void main_multiple_template_params() {
	auto m = ::max<int, double, double>(4, 7.2); // specify the template argument list explicitly
	cout << m << endl;

	system("pause");
}
