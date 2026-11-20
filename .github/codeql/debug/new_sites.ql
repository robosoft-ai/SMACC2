/**
 * @name Every new-expression CodeQL extracted from the SMACC2 sources
 * @description One row per new-expression: where it is, which allocation function it uses, and
 *              whether it is in plain code, a template instantiation or an uninstantiated template.
 * @kind table
 * @id smacc2/debug/new-sites
 */

import cpp

from NewOrNewArrayExpr n, string allocator, string where_
where
  n.getFile()
      .getRelativePath()
      .regexpMatch("(smacc2|smacc2_client_library|\\.github/codeql)/.*") and
  (
    if exists(n.getAllocator())
    then allocator = n.getAllocator().getQualifiedName()
    else allocator = "no allocator"
  ) and
  (
    if n.isFromTemplateInstantiation(_)
    then where_ = "instantiation"
    else
      if n.isFromUninstantiatedTemplate(_)
      then where_ = "uninstantiated template"
      else where_ = "plain"
  )
select n.getFile().getRelativePath() as file, n.getLocation().getStartLine() as line, allocator,
  where_, count(NewOrNewArrayExpr m | m.getLocation() = n.getLocation()) as copies
