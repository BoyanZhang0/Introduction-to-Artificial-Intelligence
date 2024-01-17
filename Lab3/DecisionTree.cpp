﻿#include "DecisionTree.h"
#include "Dataset.h"
#include "util.h"
#include <cmath>
#include <iostream>
#include<ranges>
#include<algorithm>

namespace rng = std::ranges;

DecisionTree::DecisionTree(Dataset trainset)
{
    std::cout << "decision tree learning start..." << std::endl;
	this->root = learn(trainset.examples, trainset.attributes, {});
    std::cout << "decision tree learning finished." << std::endl;
}

TreeNode* DecisionTree::plurality_value(vector<Example> examples)
{
    int pos = 0;
    for (Example example : examples)
    {
        if (example.label) pos++;
    }
    int neg = examples.size() - pos;
    bool is_pos = pos >= neg;

    TreeNode* result = new TreeNode(is_pos);

    return result;
}

vector<Example> DecisionTree::get_examples(vector<Example> examples, string attr, string option)
{
    vector<Example> result;
    int attr_index = util::get_attribute_index(attr);
    for (Example example : examples)
    {
        if (example.data[attr_index] == option) result.push_back(example);
    }
    return result;
}

int DecisionTree::get_positive_count(vector<Example>& examples)
{
    int pos = 0;
    for (Example example : examples)
    {
        if (example.label) pos++;
    }
    return pos;
}

bool DecisionTree::have_same_class(vector<Example> examples)
{
    bool label = examples[0].label;
    for (Example example : examples)
    {
        if (example.label != label) return false;
    }
    return true;
}

double DecisionTree::entropy(double p1, double p2)
{
    if (p1 == 0 || p2 == 0) return 0;
    double r1 = p1 * std::log(p1);
    double r2 = p2 * std::log(p2);
    return -(r1 + r2);
}

double DecisionTree::entropy_binary(double p)
{
    return entropy(p, 1 - p);
}

double DecisionTree::entropy_remain(string attribute, vector<Example> examples)
{
    double result = 0.0;

    map<string, set<string>> attributes_options = util::get_attributes_options();
    int attr_index = util::get_attribute_index(attribute);

    for (string option : attributes_options[attribute])
    {
        vector<Example> exs = get_examples(examples, attribute, option);
        int pos = get_positive_count(exs);
        int neg = exs.size() - pos;
        double p_k = (double)exs.size() / (double)examples.size();
        double e_k = entropy_binary((double)pos / (double)(pos + neg));
        result += p_k * e_k;
    }

    return result;
}

double DecisionTree::info_gain(string attribute, vector<Example> examples)
{
    /*
    * TODO:
    * 根据18.3.4 Choosing attribute tests中的信息增益公式计算attribute在examples上的信息增益
    * 公式：
    * Gain(attribute) = entropy_binary(p / (p + n)) - entropy_remain(attribute, examples)
    */

    int p = 0;
    for (int i = 0; i < examples.size(); i++)
    {
        if (examples[i].label == 1) p++;
    }

    return entropy_binary(p * 1.0 / (p + examples.size())) - entropy_remain(attribute, examples);
}

string DecisionTree::importance(vector<string> attributes, vector<Example> examples)
{
    /*
    * TODO:
    * 对于attributes中的每一个attribute，计算在examples上的信息增益，返回信息增益最大的attribute.
    * 例：
    * Dataset trainset("./dataset/train.csv")
    * string attr = importance(trainset.attributes, trainset.examples)
    * 此时attr应该为"PATRONS"
    * Note: attributes.size() > 0
    */

    double temp = info_gain(attributes[0], examples);
    int current = 0;
    for (int i = 1; i < attributes.size(); i++)
    {
        if (info_gain(attributes[i], examples) >= temp)
        {
            temp = info_gain(attributes[i], examples);
            current = i;
        }
    }
    return attributes[current];
}

vector<string> DecisionTree::remove_attribute(vector<string> attributes, string attribute)
{
    vector<string> result;
    for (string attr : attributes)
    {
        if (attr == attribute) continue;
        result.push_back(attr);
    }
    return result;
}

TreeNode* DecisionTree::learn(vector<Example> examples, vector<string> attributes, vector<Example> parent_examples)
{
    /*
    * TODO:
    * 实现Figure 18.5中描述的伪代码：DECISION-TREE-LEARNING
    * function DECISION-TREE-LEARNING(examples, attributes, parent_examples) returns a tree
        if examples is empty then return PLURALITY-VALUE(parent examples)
        else if all examples have the same classification then return the classification
        else if attributes is empty then return PLURALITY-VALUE(examples)
        else
            let A = the most important attribute # use importance(attributes, examples)
            let tree = a new decision tree with root test A
            for each value v_k of A do
                let exs = {e : e in examples and e.A = v_k}
                let subtree = DECISION-TREE-LEARNING(exs, {attributes − A}, examples)
                add a branch to tree with label (A = v_k) and subtree subtree # tree->options[v_k] = subtree
        return tree
    * NOTE: 如果信息增益函数正确，通过训练集得到的决策树的第一个分类属性应该为PATRONS
    */

    if (examples.empty()) return plurality_value(parent_examples);
    else if (have_same_class(examples)) return new TreeNode(examples[0].label);
    else if (attributes.empty()) return plurality_value(parent_examples);
    else
    {
        auto A = importance(attributes, examples);
        auto tree = new TreeNode(A);
        auto temp = util::get_attributes_options()[A];

        for (auto it = temp.begin(); it != temp.end(); it++)
        {
            vector<Example>exs = get_examples(examples, A, *it);
            TreeNode* subtree = learn(exs, remove_attribute(attributes, A), examples);
            tree->options[*it] = subtree;
        }
        return tree;
    }
}

int DecisionTree::classify_rec(Example& example, TreeNode* root)
{
    /*
    * TODO:
    * 实现决策树的递归预测函数
    * Algorithm:
    * function classify_rec(example, root) return an integer which indicates the label of the given example
    *   if root->attribute == util::LABEL_ATTRIBUTE then return root->value;
    * 
    *   let index = the index of root->attribute    # use util::get_attribute_index(attribute)
    *   let v_k = example.data[index]
    *   let child = root->options[v_k]
    *   return classify_rec(example, child)
    */
    if (root->attribute == util::LABEL_ATTRIBUTE) return root->value;

    auto index = util::get_attribute_index(root->attribute);
    auto v_k = example.data[index];
    auto child = root->options[v_k];
    return classify_rec(example, child);
}

vector<int> DecisionTree::classify(vector<vector<string>> test_raw_values)
{
    vector<int> result;

    Dataset testset(test_raw_values);
    for (Example example : testset.examples)
    {
        result.push_back(classify_rec(example, root));
    }

    return result;
}
