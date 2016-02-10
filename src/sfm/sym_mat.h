/*
 * sym_mat.h : This file contains some simple macros for dealing with
 * matrices stored in row major, lower triangular form.
 */

/*
 * make_sym_matrix : allocates the space required for a symmetric matrix
 * of dimension n.
 */

#define make_sym_matrix(n) \
  ((double*) malloc ((((n) * ((n) + 1)) / 2) * sizeof (double)))

/*
 * elt calculates the index of an element in a symmetric matrix stored in 
 * row major, lower triangular form.
 * N.B. : There is a difference beetween lower triangular and symmettric,
 *   which you must respect
 */

#define elt(row,col)  ((col) > (row) ? ((((col)*(col) + (col))/2) + (row)) : \
                                       ((((row)*(row) + (row))/2) + (col)))
