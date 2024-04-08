#ifndef ARC_TF_TREE_NODE_H_
#define ARC_TF_TREE_NODE_H_

#include <string>
#include <map>
#include <vector>
#include <sstream>
#include <algorithm>

namespace ARC_TF
{
    struct TreeNode {
        std::string value;
        std::vector<TreeNode*> children;
        TreeNode* parent;
    };

    class Tree {
    private:
        std::map<std::string, TreeNode*> nodes; // Tree的本體

    public:
        Tree() {}
        void removeChild(const std::string& parentValue, const std::string& childValue)
        {
            auto parentIt = nodes.find(parentValue);
            auto childIt = nodes.find(childValue);

            // 確保父節點和子節點都存在
            if (parentIt == nodes.end() || childIt == nodes.end()) {
                return;
            }
            auto& children = parentIt->second->children;
            auto childToRemove = childIt->second;

            auto it = std::find(children.begin(), children.end(), childToRemove);
            nodes[parentValue]->children.erase(it);
        }

        void addChild(const std::string& parentValue, const std::string& childValue)
        {
            TreeNode* parent = nullptr;
            if (nodes.find(parentValue) == nodes.end())
            {
                parent = new TreeNode();
                parent->value = parentValue;
                nodes[parentValue] = parent;
            }
            else {
                parent = nodes[parentValue];
            }

            TreeNode* child = nullptr;

            if (nodes.find(childValue) != nodes.end())
            {
                child = nodes[childValue];
                if (child->parent != nullptr && child->parent != parent)
                {
                    this->removeChild(child->parent->value, childValue);
                }
            }
            else
            {
                child = new TreeNode();
                child->value = childValue;
                nodes[childValue] = child;
            }

            child->parent = parent; // 設置父節點的指標
            parent->children.push_back(child); // 改為使用指標存取
        }

        std::vector<TreeNode*> getRoots()
        {
            std::vector<TreeNode*> roots;
            for (const auto& pair : nodes)
            {
                if (pair.second->parent == nullptr)
                {
                    roots.push_back(pair.second);
                }
            }
            return roots;
        }

        // 將樹狀結構轉換為字串
        std::string toString()
        {
            std::stringstream ss;

            auto roots = getRoots();
            for (const auto root : roots)
            {
                treeToStringHelper(root, ss);
            }

            return ss.str();
        }

        // 程式結束時釋放動態配置的記憶體
        ~Tree() {
            for (auto& pair : nodes)
            {
                delete pair.second;
            }
        }

    private:
        void treeToStringHelper(TreeNode* node, std::stringstream& ss, int level = 0)
        {
            for (int i = 0; i < level; ++i)
            {
                ss << "  ";
            }
            ss << "|-" << node->value << std::endl;

            for (const auto& child : node->children)
            {
                treeToStringHelper(child, ss, level + 1);
            }
        }
    };
}

#endif 
