#ifndef LAYEREDDAGSOLVER_HPP
#define LAYEREDDAGSOLVER_HPP


#include <inttypes.h>
#include <vector>
#include <algorithm>
#include <cassert>
#include <limits>
#include <iostream>

template<typename COST = uint16_t, int NUM_LAYERS = 150, int LAYER_DIM1= 2048, int LAYER_DIM2 = 32, int DELTA_DIM1 = 5, int DELTA_DIM2 = 5>
class LayeredDAGSolver
{
public:
    LayeredDAGSolver( const std::vector<COST>& data_cost_map, const std::vector<COST>& edge_cost_map ):
        _data_cost_map( data_cost_map ),
        _edge_cost_map( edge_cost_map ),
        _cumulated_cost_map( layer_size() * NUM_LAYERS, std::numeric_limits<COST>::max()/8 )
    {
        assert( _data_cost_map.size() == layer_size() * NUM_LAYERS );
        assert( _edge_cost_map.size() == edge_out_degree() );

        int cost_idx = 0;
        for( int i=-DELTA_DIM1; i<DELTA_DIM1+1; i++ )
            for( int j=-DELTA_DIM2; j<DELTA_DIM2+1; j++ )
            {
                _transition_table.push_back( std::make_pair( layer_size() + i + j*dim1(), edge_cost_map[cost_idx] ) );
                cost_idx++;
            }

        assert( _transition_table.size() == edge_out_degree() );

    }

    void expand()
    {
        // 0 first layer
        std::copy( _data_cost_map.begin(), _data_cost_map.begin() + layer_size(), _cumulated_cost_map.begin() );

        auto data_cost = _data_cost_map.begin();
        auto expanded_node = _cumulated_cost_map.begin();

        for( ; expanded_node != _cumulated_cost_map.end() - 2*layer_size(); expanded_node++, data_cost++ )
        {
            for( int i=0; i<edge_out_degree(); i++)
            {
                const auto& edge  = _transition_table[i];

                int edge_index_delta;
                COST edge_cost;
                std::tie( edge_index_delta, edge_cost ) = edge;

                COST new_cost = *expanded_node + edge_cost + *data_cost;
                auto discovererd_cell_iterator = expanded_node + edge_index_delta;

                // std::cout << "expanded " << *expanded_node << " discovered " << (*discovererd_cell_iterator) << " edge " << edge_cost << " data " << *data_cost << " new " << new_cost << "\n";

                if( *discovererd_cell_iterator > new_cost )
                {
                    // std::cout << " set " << new_cost << "\n";
                    *discovererd_cell_iterator = new_cost;
                }
            }
        }
    }

    void reconstruct_index( int cost_array_index, int& layer, int& dim1_v, int& dim2_v )
    {
        layer = cost_array_index / layer_size();
        int remainder = cost_array_index % layer_size();
        dim2_v  = remainder / dim1();
        dim1_v  = remainder % dim1();
    }

    void unroll()
    {
    //        for( auto v : _cumulated_cost_map )
    //                   std::cout << v << "\n";

        // find best node in last layer
        auto path_it = std::min( _cumulated_cost_map.end() - 2*layer_size(), _cumulated_cost_map.end() - layer_size() );

        std::cout << "# minimum cost: " << *path_it << "\n";
        int layer, dim1, dim2;
        reconstruct_index( path_it - _cumulated_cost_map.begin(), layer, dim1, dim2 );
        std::cout << *path_it << " " << layer << " " << dim1 << " " << dim2 << "\n";


        for( int layer = num_layers() - 4; layer > 0; layer-- )
        {
            auto best_predecessor      = _cumulated_cost_map.end();
            COST best_predecessor_cost = std::numeric_limits<COST>::max();

            for( auto edge : _transition_table )
            {
                auto pred_candidate = path_it - edge.first;
                if( *pred_candidate < best_predecessor_cost )
                {
                    best_predecessor = pred_candidate;
                    best_predecessor_cost = *pred_candidate;
                }
            }
            path_it = best_predecessor;
            int layer, dim1, dim2;
            reconstruct_index( path_it - _cumulated_cost_map.begin(), layer, dim1, dim2 );
            std::cout << *path_it << " " << layer << " " << dim1 << " " << dim2 << "\n";
        }
    }

    static int layer_size() { return LAYER_DIM1*LAYER_DIM2; }
    static int num_nodes()  { return NUM_LAYERS * layer_size(); }
    static int num_layers() { return NUM_LAYERS; }
    static int dim1()       { return LAYER_DIM1; }
    static int dim2()       { return LAYER_DIM2; }
    static int max_delta_dim1()  { return DELTA_DIM1; }
    static int max_delta_dim2()  { return DELTA_DIM2; }
    static int edge_out_degree() { return ( 2*DELTA_DIM1 + 1)*( 2*DELTA_DIM2 + 1); }

    std::vector<COST> _data_cost_map;
    std::vector<COST> _edge_cost_map;
    std::vector<COST> _cumulated_cost_map;

    std::vector< std::pair<int, COST> > _transition_table;
};


#endif // LAYEREDDAGSOLVER_HPP
