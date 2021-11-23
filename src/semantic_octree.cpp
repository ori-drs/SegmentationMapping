#include "semantic_octree.hpp"

namespace SegmentationMapping
{

    // --------------------
    // NODE IMPLEMENTATION
    // --------------------
    SemanticOcTreeNode::Color SemanticOcTreeNode::getAverageChildColor() const
    {

        int mr = 0;
        int mg = 0;
        int mb = 0;
        int c = 0;

        if (children != NULL)
        {
            for (int i = 0; i < 8; i++)
            {
                SemanticOcTreeNode *child = static_cast<SemanticOcTreeNode *>(children[i]);

                if (child != NULL && child->isColorSet())
                {
                    mr += child->getColor().r;
                    mg += child->getColor().g;
                    mb += child->getColor().b;
                    ++c;
                }
            }
        }

        if (c > 0)
        {
            mr /= c;
            mg /= c;
            mb /= c;
            return Color((uint8_t)mr, (uint8_t)mg, (uint8_t)mb);
        }
        else
        { // no child had a color other than white
            return Color(255, 255, 255);
        }
    }

    SemanticOcTreeNode::Color SemanticOcTreeNode::getAverageChildColor(const std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> &_label2color_map) const
    {
        if (_label2color_map.size() == 0)
        {
            int mr = 0;
            int mg = 0;
            int mb = 0;
            int c = 0;

            if (children != NULL)
            {
                for (int i = 0; i < 8; ++i)
                {
                    SemanticOcTreeNode *child = static_cast<SemanticOcTreeNode *>(children[i]);

                    if (child != NULL && child->isColorSet())
                    {
                        mr += child->getColor().r;
                        mg += child->getColor().g;
                        mb += child->getColor().b;
                        ++c;
                    }
                }
            }

            if (c > 0)
            {
                mr /= c;
                mg /= c;
                mb /= c;
                return Color((uint8_t)mr, (uint8_t)mg, (uint8_t)mb);
            }
            else
            { // no child had a color other than white
                return Color(255, 255, 255);
            }
        }
        else
        {
            // use provided color map
            auto &distribution = semantics_.label_;

            if (distribution.size() == 0)
            {
                return Color(255, 255, 255);
            }
            int max_prob_class = std::distance(distribution.begin(),
                                               std::max_element(distribution.begin(), distribution.end()));

            std::tuple<uint8_t, uint8_t, uint8_t> color_max = _label2color_map.at(max_prob_class);

            return Color(std::get<0>(color_max),
                         std::get<1>(color_max),
                         std::get<2>(color_max));
        }
    }

    SemanticOcTreeNode::Semantics SemanticOcTreeNode::getAverageChildSemantics() const
    {
        std::vector<float> mlabel;
        int c = 0;

        if (children != NULL)
        {
            for (int i = 0; i < 8; ++i)
            {
                SemanticOcTreeNode *child = static_cast<SemanticOcTreeNode *>(children[i]);

                if (child != NULL && child->isSemanticsSet())
                {
                    std::vector<float> clabel = child->getSemantics().label_;
                    // if (mlabel.empty())
                    //   mlabel.resize(clabel.size());
                    // else
                    if (mlabel.size() < clabel.size())
                        mlabel.resize(clabel.size());

                    for (int l = 0; l < (int)clabel.size(); ++l)
                    {
                        mlabel[l] += clabel[l];
                    }
                    ++c;
                }
            }
        }

        if (c > 0)
        {
            float sums = 0;
            for (int l = 0; l < (int)mlabel.size(); ++l)
            {
                mlabel[l] /= c;
                sums += mlabel[l];
            }
            // normalize
            for (auto &&d : mlabel)
                d /= sums;

            return Semantics(mlabel);
        }
        else
        { // no child had a semantics other than empty
            return Semantics();
        }
    }

    void SemanticOcTreeNode::updateColorChildren()
    {
        color_ = getAverageChildColor();
    }

    void SemanticOcTreeNode::updateColorChildren(const std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> &_label2color_map)
    {
        color_ = getAverageChildColor(_label2color_map);
    }

    void SemanticOcTreeNode::updateSemanticsChildren()
    {
        semantics_ = getAverageChildSemantics();
    }

    void SemanticOcTreeNode::normalizeSemantics()
    {
        float sum = 0;
        for (int i = 0; i < (int)semantics_.label_.size(); ++i)
            sum += semantics_.label_[i];
        if (sum > 0)
        {
            for (int i = 0; i < (int)semantics_.label_.size(); ++i)
                semantics_.label_[i] = semantics_.label_[i] / sum;
        }
        else
        {
            for (int i = 0; i < (int)semantics_.label_.size(); ++i)
                semantics_.label_[i] = 1.0 / semantics_.label_.size();
        }
    }

    // file I/O
    std::ostream &SemanticOcTreeNode::writeData(std::ostream &s) const
    {
        s.write((const char *)&value, sizeof(value)); // occupancy
        s.write((const char *)&color_, sizeof(Color)); // color

        return s;
    }

    std::istream &SemanticOcTreeNode::readData(std::istream &s)
    {
        s.read((char *)&value, sizeof(value)); // occupancy
        s.read((char *)&color_, sizeof(Color)); // color

        return s;
    }

    // --------------------
    // TREE IMPLEMENTATION
    // --------------------
    SemanticOcTree::SemanticOcTree(double _resolution)
        : octomap::OccupancyOcTreeBase<SemanticOcTreeNode>(_resolution)
    {
        semanticOcTreeMemberInit.ensureLinking();
    }

    SemanticOcTree::SemanticOcTree(double _resolution,
                                   int _num_classes,
                                   const std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> &_label2color_map)
        : octomap::OccupancyOcTreeBase<SemanticOcTreeNode>(_resolution),
          label2color_map_(_label2color_map),
          num_classes_(_num_classes)
    {
        semanticOcTreeMemberInit.ensureLinking();
    }

    bool SemanticOcTree::pruneNode(SemanticOcTreeNode *_node)
    {
        if (!isNodeCollapsible(_node))
            return false;

        // set value to children's values (all assumed equal)
        _node->copyData(*(getNodeChild(_node, 0)));

        if (_node->isColorSet()) // TODO check
            _node->setColor(_node->getAverageChildColor());

        // TODO: need to fix this
        // if (_node->isSemanticsSet())
        //     _node->setSemantics(_node->getAverageChildSemantics());

        // delete children
        for (unsigned int i = 0; i < 8; ++i)
        {
            deleteNodeChild(_node, i);
        }
        delete[] _node->children;
        _node->children = NULL;

        return true;
    }

    bool SemanticOcTree::isNodeCollapsible(const SemanticOcTreeNode *_node) const
    {
        // all children must exist, must not have children of
        // their own and have the same occupancy probability
        if (!nodeChildExists(_node, 0))
            return false;

        const SemanticOcTreeNode *firstChild = getNodeChild(_node, 0);
        if (nodeHasChildren(firstChild))
            return false;

        for (unsigned int i = 1; i < 8; ++i)
        {
            // compare nodes only using their occupancy, ignoring color for pruning
            if (!nodeChildExists(_node, i) || nodeHasChildren(getNodeChild(_node, i)) || !(getNodeChild(_node, i)->getValue() == firstChild->getValue()))
                return false;
        }

        return true;
    }

    void SemanticOcTree::averageNodeColor(SemanticOcTreeNode *_n,
                                          uint8_t _r,
                                          uint8_t _g,
                                          uint8_t _b)
    {
        if (_n != 0)
        {
            if (_n->isColorSet())
            {
                SemanticOcTreeNode::Color prev_color = _n->getColor();
                _n->setColor((prev_color.r + _r) / 2, (prev_color.g + _g) / 2, (prev_color.b + _b) / 2);
            }
            else
            {
                _n->setColor(_r, _g, _b);
            }
        }
    }

    void SemanticOcTree::averageNodeSemantics(SemanticOcTreeNode *_n,
                                              std::vector<float> &_label)
    {
        if (_n != 0)
        {
            if (_n->isSemanticsSet())
            {
                SemanticOcTreeNode::Semantics prev_semantics = _n->getSemantics();
                if (prev_semantics.label_.size() < _label.size())
                {
                    prev_semantics.label_.resize(_label.size());
                    // for (int i = 0; i < label.size(); i++)
                    //   prev_semantics.label[i] = 1.0 / label.size();
                }

                // std::cout<<"averageNodeSemantics: # of labels is "<<n->getSemantics().label.size()<<", the first one is "<<n->getSemantics().label[0]<<std::endl;

                // label[0] *= 0; // smaller weight for unlabeled data
                for (int i = 0; i < (int)_label.size(); ++i)
                {
                    prev_semantics.label_[i] = (prev_semantics.label_[i] * prev_semantics.count_ + _label[i]);
                    prev_semantics.label_[i] = prev_semantics.label_[i] / (prev_semantics.count_ + 1);
                    // prev_semantics.label[i] = (prev_semantics.label[i] + label[i]) / 2;
                }

                _n->setSemantics(prev_semantics);
                _n->normalizeSemantics();
                _n->addSemanticsCount();
            }
            else
            {
                // observe this cell the first time
                _n->setSemantics(_label);
                _n->normalizeSemantics();
                _n->resetSemanticsCount();
            }
        }
        // std::cout<<"averageNodeSemantics: # of labels is "<<n->getSemantics().label.size()<<", the first one is "<<n->getSemantics().label[0]<<std::endl;
    }

    void SemanticOcTree::updateInnerOccupancyRecurs(SemanticOcTreeNode *_node, unsigned int _depth)
    {
        // only recurse and update for inner nodes:
        if (nodeHasChildren(_node))
        {
            // return early for last level:
            if (_depth < this->tree_depth)
            {
                for (unsigned int i = 0; i < 8; ++i)
                {
                    if (nodeChildExists(_node, i))
                    {
                        updateInnerOccupancyRecurs(static_cast<SemanticOcTreeNode *>(getNodeChild(_node, i)), _depth + 1);
                    }
                }
            }
            _node->updateOccupancyChildren();

            _node->updateSemanticsChildren();

            if (label2color_map_.size())
                _node->updateColorChildren(label2color_map_);
            else
                _node->updateColorChildren();
        }
        else
            _node->updateColorChildren(label2color_map_);
    }

    std::ostream &operator<<(std::ostream &out,
                             SemanticOcTreeNode::Color const &c)
    {
        return out << '(' << (unsigned int)c.r << ' ' << (unsigned int)c.g << ' ' << (unsigned int)c.b << ')';
    }

    std::ostream &operator<<(std::ostream &out,
                             SemanticOcTreeNode::Semantics const &s)
    {
        for (unsigned i = 0; i < s.label_.size(); i++)
        {
            out << s.label_[i] << ' ';
        }
        return out;
    }

    SemanticOcTree::StaticMemberInitializer SemanticOcTree::semanticOcTreeMemberInit;

} // namespace