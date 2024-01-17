#include "search.h"
#include "inference.h"
#include <iostream>
#include <ranges>
#include <cstdlib>
#include <random>
#include <map>
namespace rng = std::ranges;

using namespace std;

// Ŀǰ��������򵥵�ѡ��ʽ������variables��˳��ѡ��
Queen* search::selectUnassignedVariable(Csp& csp)
{
	for (Queen* q : csp.variables)
	{
		if (q->position == Position::getUnassigned())
		{
			return q;
		}
	}

	return NULL;
}

// ��򵥵�˳�򣺰���variables����domain��˳��
std::vector<Position> search::orderDomainValues(Queen* var, std::vector<Queen*>& assignment, Csp& csp)
{
	return var->domain;
}

/*
 * Inferencing can eliminate the domain of variables according to current assignment.
 * This function return inferences which can be added to assignment.
 * Inferences means the variables whose domain size has been eliminated to 1 after inferencing.
*/
std::vector<Queen*> search::makeInference(Csp& csp, Queen* var, Position value)
{
	std::vector<Queen*> result;
	inference::ac3(csp);
	for (Queen* q : csp.variables)
	{
		if (q->domain.empty()) return std::vector<Queen*>({NULL});
		if (q->domain.size() == 1 && q->position == Position::getUnassigned())
		{
			q->assign(q->domain[0]);
			result.push_back(q);
		}
	}

	return result;
}
// an inferences which contains only one null pointer indicates failure.
bool search::failed(std::vector<Queen*>& inferences)
{
	if (inferences.size() == 1)
	{
		if (inferences[0] == NULL) return true;
	}

	return false;
}

bool search::isSolution(Csp& csp, std::vector<Queen*>& solution)
{
	bool result = true;

	for (Queen* q : solution)
	{
		if (getConflicts(csp, q->position) > 0) result = false;
	}

	return result;
}

// remove assigned value and inferences from assignment
void search::refresh(std::vector<Queen*>& assignment)
{
	auto it = assignment.begin();
	while (it != assignment.end())
	{
		Queen* current = *it;
		if (current->position == Position::getUnassigned()) it = assignment.erase(it);
		else it++;
	}
}

std::vector<Queen*> search::backtrackingSearch(Csp& csp)
{
	return backtrack(std::vector<Queen*>(), csp);
}

std::vector<Queen*> search::backtrack(std::vector<Queen*> assignment, Csp& csp)
{
	/*
	 * TODO
	 * Algorithm (Reference: Figure 6.5):
	 function BACKTRACK(assignment, csp) returns a solution, or failure
		if assignment is complete then return assignment (use this condition: assignment.size() == csp.variables.size())
		var<-SELECT-UNASSIGNED-VARIABLE(csp)
		for each value in ORDER-DOMAIN-VALUES(var, assignment, csp) do
			record csp state # csp.recode() require two variables, you need to create two local variables to store the state
			if value is consistent with assignment then
				assign value to var	# use var->assign(value)
				add var to assignment
				inferences<-INFERENCE(csp, var, value)	# use makeInference function here
				if inferences != failure then
					add inferences to assignment
					result<-BACKTRACK(assignment, csp)
					if result != failure then
						return result
			recover csp state (csp.recover)
			remove {var = value} and inferences from assignment # use refresh(assignment)
		return failure
	 */

	if (assignment.size() == csp.variables.size()) return assignment;
	auto var = selectUnassignedVariable(csp);
	for (auto&& value : orderDomainValues(var, assignment, csp))
	{
		vector<Position> tempLastPositions;
		vector<vector<Position>> tempLastDomains;
		csp.record(tempLastPositions, tempLastDomains);
		if (csp.consistent(value, assignment))
		{
			var->assign(value);
			assignment.push_back(var);
			vector<Queen*> tempInferences = makeInference(csp, var, value);
			if (!failed(tempInferences))
			{
				assignment.insert(assignment.end(), tempInferences.begin(), tempInferences.end());
				auto result = backtrack(assignment, csp);
				if (!failed(result)) return result;
			}
		}
		csp.recover(tempLastPositions, tempLastDomains);
		refresh(assignment);
		}
	return std::vector<Queen*>({NULL});
}

std::vector<Queen*> search::minConflict(Csp& csp, int maxSteps)
{
	/*
	 * TODO
	 * Algorithm (Reference: Figure 6.8):
	 function MIN-CONFLICTS(csp,max steps) returns a solution or failure
		inputs: csp, a constraint satisfaction problem
				max steps, the number of steps allowed before giving up
		current<-an initial complete assignment for csp	
		for i = 1 to max steps do
			if current is a solution for csp then # use isSolution
				print how many steps used here
				return current 
			var <- a randomly chosen conflicted variable from csp.VARIABLES # use chooseConflictVariable
			value <- the value v for var that minimizes CONFLICTS(var, v, current , csp) # use getMinConflictValue
			set var =value in current	# use var->position = value
		return failure
	 */
	vector<Queen*> current = csp.variables;

	for (int i = 1; i <= maxSteps; i++)
	{
		if (isSolution(csp, current))
		{
			cout << i << "steps are used here" << endl;
			return current;
		}
		auto var = chooseConflictVariable(csp);
		auto value = getMinConflictValue(csp, var);
		var->position = value;
	}

	return std::vector<Queen*>({NULL});
}

std::vector<Queen*> search::minConflictWrapper(Csp& csp)
{
	csp.randomAssign();
	return minConflict(csp, 200);
}

int search::getConflicts(Csp& csp, Position& position)
{
	/*
	* TODO
	* �õ�һ��position�ڵ�ǰ�����ϵĳ�ͻ����
	* ע�⣺��position��ͬһ�е�queen�ĳ�ͻ��Ӧ�ü���
	* ������
	*	0 1 0 0
		1 0 0 0
		0 0 1 0
		0 0 0 1
	* Position{0, 0}�ĳ�ͻ��Ӧ��Ϊ3����Ϊ����{0, 1},{2, 2},{3, 3}��ͻ
	* Position{1, 0}�ĳ�ͻ����Ӧ��Ϊ1����Ϊ����{0, 1}��ͻ
	*/

	int temp = 0;
	for (auto var : csp.variables)
	{
		if ((position.col != var->position.col) && csp.constraints(var->position, position))
			temp++;
	}
	return temp;
}

Queen* search::chooseConflictVariable(Csp& csp)
{
	/*
	* TODO
	* ����һ��Ŀǰ��ֵ�ĳ�ͻ������0��variable
	* ע�⣺��ͻ������0��variable�����ж������Ҫ���ѡ��
	* ������
	*	0 1 0 0
		1 0 0 0
		0 0 1 0
		0 0 0 1
	* Queen1-4�ĳ�ͻ��������0�����ѡ��һ����Ϊ�ú����ķ��ؽ��
	*/

	
	random_device rd;
	mt19937 gen{ rd() };
	Queen* res;
	vector<Queen*> temp;
	for (int i = 0; i < csp.variables.size(); i++)
	{
		if (getConflicts(csp, csp.variables[i]->position)) temp.push_back(csp.variables[i]);
	}
	rng::sample(temp, &res, 1, gen);

	return res;
}

Position search::getMinConflictValue(Csp& csp, Queen* var)
{
	/*
	* TODO
	* ����var��domian�У�����ʹ��ͻ����С��ֵ
	* ע�⣺ʹ��ͻ����С��ֵ�����ж������Ҫ���ѡ����������ѡ��������ܻ�����ֲ��ȶ��㲢�Ҹ��ȶ��㲻�ǽ�
	* ������
	*	1 1 0 0
		0 0 0 0
		0 0 1 0
		0 0 0 1
	* Queen1���ڵ�λ�õĳ�ͻ��Ϊ3������domainΪ{[0-3], 0}��{1, 0},{2, 0},{3, 0}�ĳ�ͻ����Ϊ1��
	* ��Ҫ�������ѡȡһ����Ϊ����ֵ��
	*/

	random_device rd;
	mt19937 gen{ rd() };
	multimap<int, Position> conflicts;
	pair<int, Position> res;
	rng::for_each(var->domain, [&](Position& pos) {
		conflicts.emplace(getConflicts(csp, pos), pos);
		});
	auto [first, last] = conflicts.equal_range(conflicts.begin()->first);
	sample(first, last, &res, 1, gen);
	return res.second;

}

void search::printSolution(std::vector<Queen*>& solution)
{
	if (search::failed(solution))
	{
		std::cout << "No Valid Solution!" << std::endl;
		return;
	}

	int size = (int)solution.size();
	std::vector<std::vector<bool>> grid(size, std::vector<bool>(size, false));

	for (Queen* queen : solution)
	{
		grid[queen->position.row][queen->position.col] = true;
	}

	for (int row = 0; row < size; row++)
	{
		for (int col = 0; col < size; col++)
		{
			std::cout << grid[row][col] << " ";
		}
		std::cout << "\n";
	}
}