#ifndef SEGMENTATION_MAPPING_SEMANTIC_OCTREE_H
#define SEGMENTATION_MAPPING_SEMANTIC_OCTREE_H

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <unordered_map>
#include <tuple> // for color map

namespace SegmentationMapping
{
    // forward declaration for friend
    class SemanticOcTree;

    // node definition
    class SemanticOcTreeNode : public octomap::OcTreeNode
    {
    public:
        friend class SemanticOcTree; // needs access to node children (inherited)

        class Color
        {
        public:
            Color() : r(255), g(255), b(255) {}
            Color(uint8_t _r, uint8_t _g, uint8_t _b)
                : r(_r), g(_g), b(_b) {}

            inline bool operator==(const Color &other) const
            {
                return (r == other.r) && (g == other.g) && (b == other.b);
            }

            inline bool operator!=(const Color &other) const
            {
                return (r != other.r) || (g != other.g) || (b != other.b);
            }
            uint8_t r, g, b;
        };

        class Semantics
        {
        public:
            Semantics() : label_(), count_(0) {}
            Semantics(int _num_classes)
            {
                label_.resize(_num_classes);
                for (int i = 0; i < _num_classes; ++i)
                {
                    label_[i] = 1.0 / _num_classes;
                }
                count_ = 1;
            }

            Semantics(std::vector<float> &_label)
            {
                label_ = _label;
                count_ = 1;
            }

            inline bool operator==(const Semantics &other) const
            {
                return (count_ == other.count_) && (label_ == other.label_);
            }

            inline bool operator!=(const Semantics &other) const
            {
                return (count_ != other.count_) || (label_ != other.label_);
            }

            std::vector<float> label_;
            unsigned int count_;
        };

    public:
        // initializers
        SemanticOcTreeNode() : octomap::OcTreeNode() {}

        SemanticOcTreeNode(const SemanticOcTreeNode &rhs)
            : octomap::OcTreeNode(rhs), color_(rhs.color_), semantics_(rhs.semantics_) {}

        SemanticOcTreeNode(int _num_classes) : octomap::OcTreeNode()
        {
            semantics_.label_.resize(_num_classes);
        }

        // comparison
        bool operator==(const SemanticOcTreeNode &_rhs) const
        {
            return (_rhs.value == value && _rhs.color_ == color_ && _rhs.semantics_ == semantics_);
        }

        // copy
        void copyData(const SemanticOcTreeNode &_from)
        {
            OcTreeNode::copyData(_from);
            color_ = _from.getColor();
            semantics_ = _from.getSemantics();
        }

        // getters and setters for color
        inline Color getColor() const { return color_; }
        inline void  setColor(Color _c) {color_ = _c; }
        inline void setColor(uint8_t _r, uint8_t _g, uint8_t _b)
        {
            color_ = Color(_r, _g, _b);
        }

        Color& getColor() { return color_; }

        inline bool isColorSet() const
        {// pure white is unlikely and so we check against that
            return ((color_.r != 255) || (color_.g != 255) || (color_.b != 255));
        }

        // getters and setters for semantics
        inline int getSemanticLabel() const
        {
            int label = 0;
            float max = 0;
            for (unsigned int c = 0; c < semantics_.label_.size(); ++c)
            {
                if (semantics_.label_[c] > max)
                {
                    label = c;
                    max = semantics_.label_[c];
                }
            }
            return label;
        }

        inline void setSemantics(Semantics &_s)
        {
            semantics_ = _s;
        }
        inline void setSemantics(std::vector<float> &_label)
        {
            semantics_.label_ = _label;
        }

        Semantics &getSemantics() { return semantics_; }
        inline Semantics getSemantics() const { return semantics_; }

        inline bool isSemanticsSet() const
        {
            return (semantics_.label_.size() > 0);
        }

        inline void addSemanticsCount() { semantics_.count_++; }
        inline void resetSemanticsCount() { semantics_.count_ = 1; }

        // get average colors and semantics
        SemanticOcTreeNode::Color getAverageChildColor() const;
        SemanticOcTreeNode::Color getAverageChildColor(const std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> &_label2color_map) const;
        Semantics getAverageChildSemantics() const;

        // update colors and semantics
        void updateColorChildren();
        void updateColorChildren(const std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> &_label2color_map);
        void updateSemanticsChildren();
        void normalizeSemantics();

        // file I/O
        std::istream &readData(std::istream &_s);
        std::ostream &writeData(std::ostream &_s) const;

    protected:
        Color color_;
        Semantics semantics_;
    };

    // tree definition
    class SemanticOcTree : public octomap::OccupancyOcTreeBase <SemanticOcTreeNode>
    {
    public:
        /**
         * @brief Construct a new Semantic Oc Tree object
         * 
         * @param _resolution resolution of the leaves
         */
        SemanticOcTree(double _resolution);

        /**
         * @brief Construct a new SemanticOcTree object
         * 
         * @param _resolution resolution of the leaves
         * @param _num_classes number of semantic classes
         * @param _label2color_map map from labels to colors
         */
        SemanticOcTree(double _resolution, int _num_classes,
                       const std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> &_label2color_map);

        /**
         * @brief Assigns the map from labels to colors
         * 
         * @param _label2color_map the map
         */
        void addColorMap(const std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> &_label2color_map)
        {
            label2color_map_ = _label2color_map;
        }

        /**
         * @brief Virtual constructor: creats a new object of the same type
         * (Covariant return type requires an up-to-date compiler)
         * 
         * @return SemanticOcTree* Pointer to a SemanticOcTree object
         */
        SemanticOcTree *create() const { return new SemanticOcTree(resolution); }

        /**
         * @brief Get the Tree Type object.
         * Setting this to ColorOcTree for the RViz plugin to recognize it.
         * 
         * @return std::string "ColorOcTree"
         */
        std::string getTreeType() const { return "ColorOcTree"; }

        /**
         * @brief Prunes a node when it is collapsible. This overloaded version
         * only considers the node occupancy for pruning, different colors of
         * the child nodes and the semantics are ignored.
         * 
         * @param _node the node to prune
         * @return true if pruning was successful
         * @return false if pruning was not successful
         */
        virtual bool pruneNode(SemanticOcTreeNode* _node);

        /**
         * @brief Checks if a node is collapsible or not
         * 
         * @param _node the node to check
         * @return true if the node is collapsible
         * @return false if the node is not collapsible
         */
        virtual bool isNodeCollapsible(const SemanticOcTreeNode* _node) const;

        void averageNodeColor(SemanticOcTreeNode *_n, uint8_t _r, uint8_t _g, uint8_t _b);
        void averageNodeSemantics(SemanticOcTreeNode *_n, std::vector<float> &_label);

    protected:
        void updateInnerOccupancyRecurs(SemanticOcTreeNode *_node, unsigned int _depth);

        /**
         * Static member object which ensures that this OcTree's prototype
         * ends up in the classIDMapping only once. You need this as a
         * static member in any derived octree class in order to read .ot
         * files through the AbstractOcTree factory. You should also call
         * ensureLinking() once from the constructor.
         */
        class StaticMemberInitializer
        {
        public:
            StaticMemberInitializer()
            {
                SemanticOcTree *tree = new SemanticOcTree(0.1);
                tree->clearKeyRays();
                AbstractOcTree::registerTreeType(tree);
            }

            /**
             * Dummy function to ensure that MSVC does not drop the
             * StaticMemberInitializer, causing this tree failing to register.
             * Needs to be called from the constructor of this octree.
             */
            void ensureLinking(){};
        };
        /// static member to ensure static initialization (only once)
        static StaticMemberInitializer semanticOcTreeMemberInit;

        std::unordered_map<int, std::tuple<uint8_t, uint8_t, uint8_t>> label2color_map_;
        int num_classes_;
    };

    //! user friendly output in format (r g b)
    std::ostream &operator<<(std::ostream &out,
                             SemanticOcTreeNode::Color const &c);

    std::ostream &operator<<(std::ostream &out,
                             SemanticOcTreeNode::Semantics const &s);

} // namespace

#endif