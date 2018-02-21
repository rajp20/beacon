$(document).ready(function(){
    var opennav = false;

    $("#icon-label").click(function(){
        if (opennav) {
            $("ul").slideUp(750);
            document.getElementById('nav-bar-icon').style.transform = "rotate(0deg)";
        } else {
            $("ul").slideDown(750);
            document.getElementById('nav-bar-icon').style.transform = "rotate(180deg)";
        }
        opennav = !opennav;
    });

    $("#top-nav-bar li").each(function(i) {
        $(this).delay(200 * i).fadeIn(750);
    });

    $('#title').fadeIn(2000);

    $('#home').delay(1000).fadeIn(1500);
    $('#footer').delay(2000).fadeIn(1000);

    var prev_content = "home";

    $('#navbar-home').click(function () {
        $('#navbar-' + prev_content).removeClass("active-nav-bar");
        $(this).addClass("active-nav-bar");

        if (opennav) {
            $("ul").slideUp(750);
            document.getElementById('nav-bar-icon').style.transform = "rotate(0deg)";
            opennav = false;
        }

        if (prev_content != "home") {
            $('#' + prev_content).fadeOut(500);
            $('#home').delay(499).fadeIn(500);
        }

        prev_content = "home";
    });

    $('#navbar-meetings').click(function () {
        $('#navbar-' + prev_content).removeClass("active-nav-bar");
        $(this).addClass("active-nav-bar");

        if (opennav) {
            $("ul").slideUp(750);
            document.getElementById('nav-bar-icon').style.transform = "rotate(0deg)";
            opennav = false;
        }

        if (prev_content != "experience") {
            $('#' + prev_content).fadeOut(500);
            $('#meetings').delay(499).fadeIn(500);
        }

        prev_content = "meetings";
    });

});