$ErrorActionPreference = 'Stop'
$lib = 'D:/sw2022/SOLIDWORKS/api/redist/SolidWorks.Interop.sldworks.dll'
Add-Type -Path $lib
Add-Type -ReferencedAssemblies $lib -TypeDefinition @"
using System;
using SolidWorks.Interop.sldworks;
public static class BoxExport {
 public static object[] Run(string input, string output) {
  ISldWorks app = (ISldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
  IModelDoc2 doc = null;
  try {
   app.Visible = false;
   int errors=0, warnings=0;
   doc = app.OpenDoc6(input,1,3,"",ref errors,ref warnings);
   if (doc == null) throw new Exception("OpenDoc6 errors="+errors+" warnings="+warnings);
   IPartDoc part = (IPartDoc)doc;
   object bounds = part.GetPartBox(true);
   IMassProperty mass = (IMassProperty)doc.Extension.CreateMassProperty();
   double m = mass.Mass;
   object center = mass.CenterOfMass;
   object inertia = mass.GetMomentOfInertia(0);
   bool ok = doc.Extension.SaveAs(output,0,3,null,ref errors,ref warnings);
   if (!ok || errors != 0) throw new Exception("STL export errors="+errors+" warnings="+warnings);
   return new object[] {bounds,m,center,inertia,app.RevisionNumber(),warnings};
  } finally { if(doc != null) app.CloseDoc(doc.GetTitle()); }
 }
}
"@
$result = [BoxExport]::Run('C:/Users/Alaso/Desktop/box.SLDPRT', (Join-Path $PSScriptRoot '../meshes/box_raw.stl'))
[ordered]@{bounds_m=$result[0];cad_mass_kg=$result[1];center_of_mass_m=$result[2];inertia_kg_m2=$result[3];solidworks_revision=$result[4];export_warnings=$result[5]} | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'box_export.json') -Encoding utf8
Get-Content -LiteralPath (Join-Path $PSScriptRoot 'box_export.json')