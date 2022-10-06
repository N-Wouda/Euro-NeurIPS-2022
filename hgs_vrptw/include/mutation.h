#ifndef MUTATION_H_
#define MUTATION_H_

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"

#include <vector>

/**
 * Performs a SISR mutation on the passed-in individual. SISRX first removes
 * strings of clients from based on "Slack Induction String Removals (SISRs)"
 * [1]. The individual is then repaired to a complete solution by greedily
 * re-inserting the unplanned clients.
 * <br />
 * [1]: Christiaens, J., & Vanden Berghe, G. (2020). Slack induction by string
 * removals for vehicle routing problems. Transportation Science, 54(2),
 * 417-433.
 */
Individual
stringRemovals(Individual &indiv, Params const &params, XorShift128 &rng);

#endif  // MUTATION_H_
