#include <vector>
#include <iostream>
#include <numeric>

double scores_product(const std::vector<int>& scores) {
	return std::accumulate(scores.cbegin(), 
		scores.cend(), 
		1,
		std::multiplies<int>());
}

void main_calculate_product() {
//void main() {
	std::vector<int> scores = { 1, 2, 3, 4, 5 };
	std::cout << scores_product(scores) << std::endl;

	system("pause");
}



