#ifndef DBOW_EXTENSIONS_H
#define DBOW_EXTENSIONS_H

#include <DBoW2/FORB.h>
#include <DBoW2/TemplatedVocabulary.h>

namespace DBoW2 {
    
template<class TDescriptor, class F>
/// Generic Vocabulary
class TemplatedVocabularyWithTextIO : public TemplatedVocabulary<TDescriptor,F>
{
public:
    typedef TemplatedVocabulary<TDescriptor,F> BaseT;
    
    TemplatedVocabularyWithTextIO(int k = 10, int L = 5, 
                                  WeightingType weighting = TF_IDF, ScoringType scoring = L1_NORM)
    : BaseT(k,L,weighting, scoring)
    {
        
    }
    
    /**
     * Creates the vocabulary by loading a file
     * @param filename
     */
    TemplatedVocabularyWithTextIO(const std::string &filename) : BaseT(filename) { }
    
    /**
     * Creates the vocabulary by loading a file
     * @param filename
     */
    TemplatedVocabularyWithTextIO(const char *filename) : BaseT(filename) { }
    
    /** 
     * Copy constructor
     * @param voc
     */
    TemplatedVocabularyWithTextIO(const TemplatedVocabularyWithTextIO<TDescriptor, F> &voc) : BaseT(voc)
    {
        
    }
    
    /**
     * Destructor
     */
    virtual ~TemplatedVocabularyWithTextIO()
    {
        
    }
    
    using BaseT::operator=;
    
    bool loadFromTextFile(const std::string &filename)
    {
        std::ifstream f;
        f.open(filename.c_str());
        
        if(f.eof())
            return false;
        
        BaseT::m_words.clear();
        BaseT::m_nodes.clear();
        
        std::string s;
        getline(f,s);
        std::stringstream ss;
        ss << s;
        ss >> BaseT::m_k;
        ss >> BaseT::m_L;
        int n1, n2;
        ss >> n1;
        ss >> n2;
        
        if(BaseT::m_k<0 || BaseT::m_k>20 || BaseT::m_L<1 || BaseT::m_L>10 || n1<0 || n1>5 || n2<0 || n2>3)
        {
            std::cerr << "Vocabulary loading failure: This is not a correct text file!" << std::endl;
            return false;
        }
        
        BaseT::m_scoring = (ScoringType)n1;
        BaseT::m_weighting = (WeightingType)n2;
        BaseT::createScoringObject();
        
        // nodes
        int expected_nodes =
        (int)((pow((double)BaseT::m_k, (double)BaseT::m_L + 1) - 1)/(BaseT::m_k - 1));
        BaseT::m_nodes.reserve(expected_nodes);
        
        BaseT::m_words.reserve(pow((double)BaseT::m_k, (double)BaseT::m_L + 1));
        
        BaseT::m_nodes.resize(1);
        BaseT::m_nodes[0].id = 0;
        while(!f.eof())
        {
            std::string snode;
            getline(f,snode);
            std::stringstream ssnode;
            ssnode << snode;
            
            int nid = BaseT::m_nodes.size();
            BaseT::m_nodes.resize(BaseT::m_nodes.size()+1);
            BaseT::m_nodes[nid].id = nid;
            
            int pid ;
            ssnode >> pid;
            BaseT::m_nodes[nid].parent = pid;
            BaseT::m_nodes[pid].children.push_back(nid);
            
            int nIsLeaf;
            ssnode >> nIsLeaf;
            
            std::stringstream ssd;
            for(int iD=0;iD<F::L;iD++)
            {
                std::string sElement;
                ssnode >> sElement;
                ssd << sElement << " ";
            }
            F::fromString(BaseT::m_nodes[nid].descriptor, ssd.str());
            
            ssnode >> BaseT::m_nodes[nid].weight;
            
            if(nIsLeaf>0)
            {
                int wid = BaseT::m_words.size();
                BaseT::m_words.resize(wid+1);
                
                BaseT::m_nodes[nid].word_id = wid;
                BaseT::m_words[wid] = &BaseT::m_nodes[nid];
            }
            else
            {
                BaseT::m_nodes[nid].children.reserve(BaseT::m_k);
            }
        }
        
        return true;
        
    }
    
    void saveToTextFile(const std::string &filename) const
    {
        std::fstream f;
        f.open(filename.c_str(),std::ios_base::out);
        f << BaseT::m_k << " " << BaseT::m_L << " " << " " << BaseT::m_scoring << " " << BaseT::m_weighting << std::endl;
        
        for(size_t i=1; i<BaseT::m_nodes.size();i++)
        {
            const typename BaseT::Node& node = BaseT::m_nodes[i];
            
            f << node.parent << " ";
            if(node.isLeaf())
                f << 1 << " ";
            else
                f << 0 << " ";
            
            f << F::toString(node.descriptor) << " " << (double)node.weight << std::endl;
        }
        
        f.close();
    }
};
    
}

#endif // DBOW_EXTENSIONS_H
