using WinchControllers
using Documenter

# DocMeta.setdocmeta!(WinchControllers, :DocTestSetup; recursive=true)

makedocs(;
    modules=[WinchControllers],
    authors="Uwe Fechner <fechner@aenarete.eu>",
    repo="https://github.com/OpenSourceAWE/WinchControllers.jl/blob/{commit}{path}#{line}",
    sitename="WinchControllers.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://OpenSourceAWE.github.io/WinchControllers.jl",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
        "Components" => "components.md",
        "Types" => "types.md",
        "Functions" => "functions.md"
    ],
)

deploydocs(;
    repo="github.com/OpenSourceAWE/WinchControllers.jl",
    devbranch="main",
)
