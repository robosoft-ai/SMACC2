/**
 * @name Calls of operator new and operator delete that CodeQL holds, per file
 * @description What the 21.6.2 query has to work with: calls whose target is named operator new or
 *              operator delete, split by whether the target is a global function or a class member.
 * @kind table
 * @id smacc2/debug/operator-new-calls
 */

import cpp

string scopeOf(Function f) {
  if f instanceof MemberFunction then result = "class member" else result = "global"
}

from File file, string name, string scope, int n
where
  exists(file.getRelativePath()) and
  name = ["operator new", "operator new[]", "operator delete", "operator delete[]"] and
  scope = ["class member", "global"] and
  n =
    count(FunctionCall c |
      c.getFile() = file and c.getTarget().getName() = name and scopeOf(c.getTarget()) = scope
    ) and
  n > 0
select file.getRelativePath() as path, name, scope, n order by path, name, scope
