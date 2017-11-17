#include <string>
#include <iostream>
#include <algorithm>

bool is_not_space(char c) {
	return (c != ' ');
}

std::string trim_left(std::string s) {
	s.erase(s.begin(),
		std::find_if(s.begin(), s.end(), is_not_space));
	return s;
}

std::string trim_right(std::string s) {
	s.erase(std::find_if(s.rbegin(), s.rend(), is_not_space).base(), s.end());
	return s;
}

std::string trim(const std::string& s) {
	return trim_left(trim_right(s));
}

void main_string_trimming() {
//void main(){
	std::string test = "              abc       ";
	std::cout << "trim_left is [" << trim_left(test) << "]" << std::endl;
	std::cout << "trim_right is [" << trim_right(test) << "]" << std::endl;
	std::cout << "trim is [" << trim(test) << "]" << std::endl;

	system("pause");
}