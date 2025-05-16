using WinchControllers
using Documenter

# DocMeta.setdocmeta!(WinchControllers, :DocTestSetup; recursive=true)

makedocs(;
    modules=[WinchControllers],
    authors="Uwe Fechner <fechner@aenarete.eu>",
    repo="https://github.com/aenarete/KiteModels.jl/blob/{commit}{path}#{line}",
    sitename="WinchControllers.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://aenarete.github.io/WinchControllers.jl",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
        "Types" => "types.md",
        "Functions" => "functions.md"
    ],
)

deploydocs(;
    repo="github.com/aenarete/WinchControllers.jl",
    devbranch="main",
)
