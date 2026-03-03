# build and display the html documentation locally

using Pkg

if !("Documenter" ∈ keys(Pkg.project().dependencies))
    Pkg.activate("docs")
end
using LiveServer; servedocs(launch_browser=true)
