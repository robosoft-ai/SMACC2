/**
 * @name The new-expressions CodeQL extracted, per file
 * @description One row per file under the checkout, allocation function and kind of code: how many
 *              new-expressions CodeQL holds there. Shows where the 21.6.2 query can report at all.
 * @kind table
 * @id smacc2/debug/new-sites
 */

import cpp

string allocatorOf(NewOrNewArrayExpr e) {
  if exists(e.getAllocator())
  then result = e.getAllocator().getQualifiedName()
  else result = "no allocator"
}

string kindOf(NewOrNewArrayExpr e) {
  if e.isFromTemplateInstantiation(_)
  then result = "instantiation"
  else
    if e.isFromUninstantiatedTemplate(_)
    then result = "uninstantiated template"
    else result = "plain"
}

from File f, string allocator, string kind, int n
where
  exists(f.getRelativePath()) and
  exists(NewOrNewArrayExpr e0 |
    e0.getFile() = f and allocator = allocatorOf(e0) and kind = kindOf(e0)
  ) and
  n =
    count(NewOrNewArrayExpr e |
      e.getFile() = f and allocatorOf(e) = allocator and kindOf(e) = kind
    )
select f.getRelativePath() as file, allocator, kind, n order by file, allocator, kind
