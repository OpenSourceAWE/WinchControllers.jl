using REPL.TerminalMenus, WinchControllers

URL="https://opensourceawe.github.io/WinchControllers.jl/dev/"

options = ["test_winchcontroller = include(\"test_winchcontroller.jl\")",      
           "autotune_ = include(\"autotune.jl\")",
           "test_tuned_winchcontroller = include(\"test_tuned_winchcontroller.jl\")",
           "plot_power = include(\"plot_power.jl\")",
           "stability_ufc = include(\"stability_ufc.jl\")",
           "stability_lfc = include(\"stability_lfc.jl\")",
           "help = WinchControllers.help()",
           "quit"]

function example_menu()
    active = true
    while active
        menu = RadioMenu(options, pagesize=8)
        choice = request("\nChoose function to execute or `q` to quit: ", menu)

        if choice != -1 && choice != length(options)
            eval(Meta.parse(options[choice]))
        else
            println("Left menu. Press <ctrl><d> to quit Julia!")
            active = false
        end
    end
end

example_menu()