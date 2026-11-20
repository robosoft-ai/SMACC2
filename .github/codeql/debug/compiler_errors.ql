/**
 * @name Compiler errors CodeQL met while extracting
 * @description An error can make the extractor drop the body of a function, and with it its results.
 * @kind table
 * @id smacc2/debug/compiler-errors
 */

import cpp

from CompilerError e
select e.getLocation().toString() as place, e.getMessage() as message
