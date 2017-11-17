#include <vector>
#include <iostream>
#include <numeric>

double average_score_normal(const std::vector<int> &scores) {
	int sum = 0;
	for (int score : scores) {
		sum += score;
	}
	return sum / (double)scores.size();
}

double average_score(const std::vector<int>& scores) {
	return std::accumulate(scores.cbegin(), scores.cend(), 0) / (double)scores.size();
}

void main_calculate_average(){
//void main() {
	std::vector<int> scores = { 1, 2, 3, 4, 5 };
	std::cout << average_score_normal(scores) << std::endl;
	std::cout << average_score(scores) << std::endl;

	system("pause");
}



