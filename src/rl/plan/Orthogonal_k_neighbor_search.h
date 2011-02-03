// Copyright (c) 2002 Utrecht University (The Netherlands).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org); you may redistribute it under
// the terms of the Q Public License version 1.0.
// See the file LICENSE.QPL distributed with CGAL.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL: svn+ssh://scm.gforge.inria.fr/svn/cgal/branches/CGAL-3.3-branch/Spatial_searching/include/CGAL/Orthogonal_k_neighbor_search.h $
// $Id: Orthogonal_k_neighbor_search.h 36334 2007-02-15 21:24:48Z spion $
// 
//
// Author(s)     : Hans Tangelder (<hanst@cs.uu.nl>)
//                 Markus Rickert

#ifndef _RL_PLAN_ORTHOGONAL_K_NEIGHBOR_SEARCH_H_
#define _RL_PLAN_ORTHOGONAL_K_NEIGHBOR_SEARCH_H_

#include <cstring>
#include <list>
#include <memory>
#include <queue>
#include <CGAL/basic.h>
#include <CGAL/Euclidean_distance.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Kd_tree_node.h>
#include <CGAL/Splitters.h>

namespace rl
{
	namespace plan
	{
		template<
			typename SearchTraits,
			typename Distance_ = ::CGAL::Euclidean_distance<SearchTraits>,
			typename Splitter_ = ::CGAL::Sliding_midpoint<SearchTraits>,
			typename Tree_ = ::CGAL::Kd_tree<SearchTraits, Splitter_, ::CGAL::Tag_true>
		>
		class Orthogonal_k_neighbor_search
		{	
		public:
			typedef Distance_ Distance;
			
			typedef Splitter_ Splitter;
			
			typedef Tree_ Tree;
			
			typedef typename SearchTraits::FT FT;
			
			typedef typename Tree::Node_handle Node_handle;
			
			typedef typename Distance::Query_item Query_item;
			
			typedef typename SearchTraits::Point_d Point_d;
			
			typedef typename Tree::Point_d_iterator Point_d_iterator;
			
			typedef ::std::pair< Point_d, FT > Point_with_transformed_distance;
			
		private:
			typedef ::std::list< Point_with_transformed_distance > NN_list;
			
		public:
			typedef typename NN_list::const_iterator iterator;
			
			Orthogonal_k_neighbor_search(
				Tree& tree,
				const Query_item& q,
				int k = 1,
				FT Eps = FT(0),
				bool Search_nearest = true,
				const Distance& d = Distance()
			) :
				actual_k(0),
				distance_instance(d),
				distance_to_root(),
				l(),
				max_k(k),
				multiplication_factor(d.transformed_distance(1 + Eps)),
				number_of_internal_nodes_visited(0),
				number_of_items_visited(0),
				number_of_leaf_nodes_visited(0),
				query_object(q),
				search_nearest(Search_nearest),
				total_item_number(tree.size())
			{
				if (search_nearest)
				{
					distance_to_root = d.min_distance_to_rectangle(q, tree.bounding_box());
				}
				else
				{
					distance_to_root = d.max_distance_to_rectangle(q, tree.bounding_box());
				}
				
				compute_neighbors_orthogonally(tree.root(), distance_to_root);
			}
			
			iterator begin() const
			{
				return l.begin();
			}
			
			iterator end() const
			{
				return l.end();
			}
			
			::std::ostream& statistics(::std::ostream& s)
			{
				s << "K_Neighbor search statistics:" << ::std::endl;
				s << "Number of internal nodes visited:" << number_of_internal_nodes_visited << ::std::endl;
				s << "Number of leaf nodes visited:" << number_of_leaf_nodes_visited << ::std::endl;
				s << "Number of items visited:" << number_of_items_visited << ::std::endl;
				return s;
			}
			
		protected:
			
		private:
			bool branch(FT distance)
			{
				if (actual_k < max_k)
				{
					return true;
				}
				else
				{
					if (search_nearest)
					{
						return distance * multiplication_factor < l.rbegin()->second;
					}
					else
					{
						return distance > l.begin()->second * multiplication_factor;
					}
				}
			}
			
			void compute_neighbors_orthogonally(Node_handle N, FT rd)
			{
				typename SearchTraits::Construct_cartesian_const_iterator_d construct_it;
				
				typename SearchTraits::Cartesian_const_iterator_d query_object_it = construct_it(query_object);
				
				if (!(N->is_leaf()))
				{
					++number_of_internal_nodes_visited;
					
					FT low_off = distance_instance.min_distance_to_rectangle(
						*(query_object_it + N->cutting_dimension()),
						N->low_value(),
						N->cutting_value(),
						N->cutting_dimension()
					);
					
					FT high_off = distance_instance.min_distance_to_rectangle(
						*(query_object_it + N->cutting_dimension()),
						N->cutting_value(),
						N->high_value(),
						N->cutting_dimension()
					);
					
					FT new_rd;
					
					if ((low_off < high_off && search_nearest) || (low_off >= high_off && !search_nearest))
					{
						compute_neighbors_orthogonally(N->lower(), rd);
						
						if (search_nearest)
						{
							new_rd = distance_instance.new_distance(rd, low_off, high_off, N->cutting_dimension());
						}
						else
						{
							new_rd = distance_instance.new_distance(rd, high_off, low_off, N->cutting_dimension());
						}
						
						if (branch(new_rd))
						{
							compute_neighbors_orthogonally(N->upper(), new_rd);
						}
					}
					else
					{
						compute_neighbors_orthogonally(N->upper(), rd);
						
						if (search_nearest)
						{
							new_rd = distance_instance.new_distance(rd, high_off, low_off, N->cutting_dimension());
						}
						else
						{
							new_rd = distance_instance.new_distance(rd, low_off, high_off, N->cutting_dimension());
						}
						
						if (branch(new_rd))
						{
							compute_neighbors_orthogonally(N->lower(), new_rd);
						}
					}
				}
				else
				{
					++number_of_leaf_nodes_visited;
					
					if (N->size() > 0)
					{
						for (Point_d_iterator it = N->begin(); it != N->end(); ++it)
						{
							++number_of_items_visited;
							FT distance_to_query_object = distance_instance.transformed_distance(query_object, **it);
							insert(*it, distance_to_query_object);
						}
					}
				}
			}
			
			void insert(Point_d* I, FT dist)
			{
				bool insert;
				
				if (actual_k < max_k)
				{
					insert = true;
				}
				else
				{
					if (search_nearest)
					{
						insert = dist < l.rbegin()->second;
					}
					else
					{
						insert = dist > l.rbegin()->second;
					}
				}
				
				if (insert)
				{
					++actual_k;
					
					typename NN_list::iterator it = l.begin();
					
					if (search_nearest)
					{
						for (; it != l.end(); ++it)
						{
							if (dist < it->second)
							{
								break;
							}
						}
					}
					else
					{
						for (; it != l.end(); ++it)
						{
							if (dist > it->second)
							{
								break;
							}
						}
					}
					
					Point_with_transformed_distance NN_Candidate(*I, dist);
					
					l.insert(it, NN_Candidate);
					
					if (actual_k > max_k)
					{
						--actual_k;
						l.pop_back();
					}
				}
			}
			
			int actual_k;
			
			Distance distance_instance;
			
			FT distance_to_root;
			
			NN_list l;
			
			int max_k;
			
			FT multiplication_factor;
			
			int number_of_internal_nodes_visited;
			
			int number_of_items_visited;
			
			int number_of_leaf_nodes_visited;
			
			Query_item query_object;
			
			bool search_nearest;
			
			int total_item_number;
		};
	}
}

#endif // _RL_PLAN_ORTHOGONAL_K_NEIGHBOR_SEARCH_H_
