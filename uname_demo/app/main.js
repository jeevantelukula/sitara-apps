$(document).ready(function() {
    $("#run-uname-button").click(function() {
        $.get("/run-uname", function(data) {
            $("#output pre").text(data);
        });
    });
});