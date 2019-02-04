$(document).ready(function () {
  $(".js-toggle-item").css("display", "none");
  $(".course-part-summary__switch-icon i").css("transform", "rotate(0)");
  $(".course-part-summary__section:first-child .js-toggle-item").css("display", "block");
  $(".course-part-summary__section:first-child .course-part-summary__switch-icon i").css("transform", "rotate(90deg)");

  $(".js-toggle").click(function(){
    $(".js-toggle-item").css("display", "none");
    $(".js-toggle .course-part-summary__switch-icon i").css("transform", "rotate(0)");
    var ol = $(this).siblings('ol');
    ol.css("display", "block");
    $(this).children(".course-part-summary__switch-icon").children("i").css("transform", "rotate(90deg)");
  });
});
