#include <iostream>
#include <string>
using namespace std;

template<typename T1, typename T2>
auto max(T1 a, T2 b) { // C++14
	return b < a ? a : b;
}

//void main() {
void main_auto() {
	auto m = ::max<int, double>(4, 10.2); // specify the template argument list explicitly
	cout << m << endl;

	cout << ::max(40, 10.0) << endl;

	system("pause");
}
